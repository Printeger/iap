#ifndef _BSPLINE_OPTIMIZER_H_
#define _BSPLINE_OPTIMIZER_H_

#include <cstdint>
#include <limits>
#include <Eigen/Eigen>
#include <path_searching/dyn_a_star.h>
#include <bspline_opt/uniform_bspline.h>
#include <plan_env/grid_map.h>
#include <plan_env/obj_predictor.h>
#include <rclcpp/rclcpp.hpp>
#include <iap/planner/p1_accepted_context_validation.hpp>
#include "bspline_opt/lbfgs.hpp"
#include <traj_utils/plan_container.hpp>
#include <memory>
#include <map>
#include <string>

namespace iap
{
  class RiskGridSnapshot;
}

// Gradient and elasitc band optimization

// Input: a signed distance field and a sequence of points
// Output: the optimized sequence of points
// The format of points: N x 3 matrix, each row is a point
namespace ego_planner
{

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

  public:
    struct P1IntegrityConfig
    {
      bool use_integrity_cost = false;
      bool metrics_only = true;
      double lambda_integrity = 0.0;
      double sample_dt_min_s = 0.1;
      double sample_dt_scale = 1.0;
      int max_samples_per_eval = 30;
      double integrity_cost_max = 100.0;
      double integrity_grad_norm_max = 0.1;
      std::string unknown_policy = "skip";
      double unknown_soft_penalty = 1.0;
      bool debug_csv_enable = false;
      std::string debug_csv_path;
      // Immutable launch identity copied into every P1 artifact row.  These
      // fields deliberately travel with the optimizer rather than being
      // inferred from a "latest" export directory after the run.
      std::string evidence_schema_version;
      std::string evidence_run_id;
      std::string evidence_manifest_path;
      int max_candidates_per_attempt = 8;
      // Admission is judged on a fixed 200-sample lattice.  Keep the soft
      // objective on that same lattice so a narrow peak cannot be hidden by
      // adaptive optimizer sampling.  Mean and normalized LSE both produced
      // retained formal peak counterexamples, so the selected P1-2 mode is a
      // differentiable upper-tail CVaR on the same fixed support.
      std::string objective_aggregation_mode = "fixed_200_smooth_cvar";
      double smooth_max_temperature = 0.01;
      double smooth_cvar_alpha = 0.90;
      double normalization_budget_fraction = 0.30;
    };

    const P1IntegrityConfig &p1IntegrityConfig() const { return p1_config_; }

    struct P1IntegrityMetrics
    {
      int sample_count = 0;
      int hit_count = 0;
      int miss_count = 0;
      int stale_count = 0;
      uint64_t snapshot_generation_id = 0;
      double f_integrity = 0.0;
      double weighted_f_integrity = 0.0;
      double normalized_weighted_f_integrity = 0.0;
      double anchor_cost = 0.0;
      double grad_norm_integrity = 0.0;
      double full_grad_norm_integrity = 0.0;
      double weighted_grad_integrity_norm = 0.0;
      double normalized_weighted_grad_integrity_norm = 0.0;
      double full_normalized_weighted_grad_integrity_norm = 0.0;
      double grad_norm_original = 0.0;
      double full_grad_norm_original = 0.0;
      double full_total_gradient_norm = 0.0;
      double grad_ratio = 0.0;
      double base_p1_cosine = 0.0;
      double miss_ratio = 0.0;
      double stale_ratio = 0.0;
      double peak_contribution = 0.0;
      int clipped_grad_count = 0;
      std::string fallback_reason = "not_evaluated";
      bool applied_to_objective = false;
      uint64_t planning_attempt_id = 0;
      uint64_t candidate_id = 0;
    };

    // The planning manager creates this immutable tuple once per planning
    // attempt.  Keeping it in the optimizer binds objective/debug/profile
    // evidence to one P0 snapshot instead of whichever snapshot is newest.
    struct P1PlanningRiskContext
    {
      std::shared_ptr<const iap::RiskGridSnapshot> snapshot;
      double query_base_time_s = 0.0;
      double planning_start_s = std::numeric_limits<double>::quiet_NaN();
      uint64_t planning_attempt_id = 0;
      uint64_t candidate_id = 0;
      bool objective_allowed = true;
      std::string fallback_reason = "none";
    };

    struct OptimizerCostBreakdown
    {
      double total_cost = 0.0;
      double original_cost = 0.0;
      double integrity_cost = 0.0;
      double normalized_integrity_cost = 0.0;
      double anchor_cost = 0.0;
    };

    struct P1BasePrepassTrace
    {
      double pre_objective = std::numeric_limits<double>::quiet_NaN();
      double post_objective = std::numeric_limits<double>::quiet_NaN();
      double duration_ms = 0.0;
      int solver_result = 0;
      int iteration_count = 0;
      bool success = false;
      std::string termination_reason = "not_run";
    };

    struct P1OptimizerCheckpoint
    {
      std::string stage = "p1_stage";
      std::string checkpoint = "not_recorded";
      int restart_index = 0;
      int iteration = 0;
      int line_search_count = 0;
      double step = 0.0;
      double objective = std::numeric_limits<double>::quiet_NaN();
      double base_objective = std::numeric_limits<double>::quiet_NaN();
      double raw_p1_objective = std::numeric_limits<double>::quiet_NaN();
      double normalized_p1_objective = 0.0;
      double anchor_objective = 0.0;
      double x_norm = std::numeric_limits<double>::quiet_NaN();
      double gradient_norm = std::numeric_limits<double>::quiet_NaN();
      double directional_derivative = std::numeric_limits<double>::quiet_NaN();
      int solver_result = 0;
      std::string reason = "none";
    };

    struct P1FixedLatticeRiskSummary
    {
      bool full_support = false;
      int valid_sample_count = 0;
      int occupied_sample_count = 0;
      int evidence_miss_count = 0;
      double mean_c_pi = std::numeric_limits<double>::quiet_NaN();
      double max_c_pi = std::numeric_limits<double>::quiet_NaN();
    };

    struct P1FanoutDiagnostics
    {
      int input_topology_segments = 0;
      int surviving_topology_segments = 0;
      int returned_candidate_count = 0;
      int configured_cap = 8;
      int optimizer_success_count = 0;
      int full_support_count = 0;
      int p1_descent_eligible_count = 0;
      int supplemental_candidate_count = 0;
      bool truncated = false;
      bool singleton_due_to_empty_segments = false;
      bool singleton_due_to_degenerate_segments = false;
      bool singleton_due_to_opposite_direction_unavailable = false;
      std::string optimizer_selected_candidate = "";
      std::string replacement_acceptance = "not_evaluated";
    };

    // A compact, test-facing record of one optimizer invocation.  This is
    // intentionally aggregate evidence: the accepted-profile CSV remains the
    // authoritative per-sample record.
    struct P1OptimizationTrace
    {
      double pre_base_objective = std::numeric_limits<double>::quiet_NaN();
      double post_base_objective = std::numeric_limits<double>::quiet_NaN();
      double pre_total_objective = std::numeric_limits<double>::quiet_NaN();
      double post_total_objective = std::numeric_limits<double>::quiet_NaN();
      double raw_p1_cost = std::numeric_limits<double>::quiet_NaN();
      double weighted_p1_cost = std::numeric_limits<double>::quiet_NaN();
      double base_gradient_norm = std::numeric_limits<double>::quiet_NaN();
      double p1_gradient_norm = std::numeric_limits<double>::quiet_NaN();
      double total_gradient_norm = std::numeric_limits<double>::quiet_NaN();
      double displacement_norm = std::numeric_limits<double>::quiet_NaN();
      double grad_integrity_dot_displacement = std::numeric_limits<double>::quiet_NaN();
      double weighted_p1_gradient_dot_displacement = std::numeric_limits<double>::quiet_NaN();
      double total_gradient_dot_displacement = std::numeric_limits<double>::quiet_NaN();
      double pre_mean_c_pi = std::numeric_limits<double>::quiet_NaN();
      double pre_max_c_pi = std::numeric_limits<double>::quiet_NaN();
      double post_mean_c_pi = std::numeric_limits<double>::quiet_NaN();
      double post_max_c_pi = std::numeric_limits<double>::quiet_NaN();
      // Preserve both sides of every quantity.  The legacy aggregate fields
      // above remain for compatibility with earlier exported CSVs.
      double pre_raw_p1_cost = std::numeric_limits<double>::quiet_NaN();
      double post_raw_p1_cost = std::numeric_limits<double>::quiet_NaN();
      double pre_weighted_p1_cost = std::numeric_limits<double>::quiet_NaN();
      double post_weighted_p1_cost = std::numeric_limits<double>::quiet_NaN();
      double pre_base_gradient_norm = std::numeric_limits<double>::quiet_NaN();
      double post_base_gradient_norm = std::numeric_limits<double>::quiet_NaN();
      double pre_full_base_gradient_norm = std::numeric_limits<double>::quiet_NaN();
      double post_full_base_gradient_norm = std::numeric_limits<double>::quiet_NaN();
      double pre_raw_p1_gradient_norm = std::numeric_limits<double>::quiet_NaN();
      double post_raw_p1_gradient_norm = std::numeric_limits<double>::quiet_NaN();
      double pre_full_raw_p1_gradient_norm = std::numeric_limits<double>::quiet_NaN();
      double post_full_raw_p1_gradient_norm = std::numeric_limits<double>::quiet_NaN();
      double pre_weighted_p1_gradient_norm = std::numeric_limits<double>::quiet_NaN();
      double post_weighted_p1_gradient_norm = std::numeric_limits<double>::quiet_NaN();
      double pre_normalized_weighted_p1_gradient_norm = std::numeric_limits<double>::quiet_NaN();
      double post_normalized_weighted_p1_gradient_norm = std::numeric_limits<double>::quiet_NaN();
      double pre_full_normalized_weighted_p1_gradient_norm = std::numeric_limits<double>::quiet_NaN();
      double post_full_normalized_weighted_p1_gradient_norm = std::numeric_limits<double>::quiet_NaN();
      double pre_total_gradient_norm = std::numeric_limits<double>::quiet_NaN();
      double post_total_gradient_norm = std::numeric_limits<double>::quiet_NaN();
      double pre_full_total_gradient_norm = std::numeric_limits<double>::quiet_NaN();
      double post_full_total_gradient_norm = std::numeric_limits<double>::quiet_NaN();
      double pre_base_p1_cosine = 0.0;
      double post_base_p1_cosine = 0.0;
      double pre_normalized_p1_cost = 0.0;
      double post_normalized_p1_cost = 0.0;
      double pre_anchor_cost = 0.0;
      double post_anchor_cost = 0.0;
      double base_prepass_pre_objective = std::numeric_limits<double>::quiet_NaN();
      double base_prepass_post_objective = std::numeric_limits<double>::quiet_NaN();
      double base_prepass_duration_ms = 0.0;
      int base_prepass_solver_result = 0;
      int base_prepass_iteration_count = 0;
      bool base_prepass_success = false;
      std::string base_prepass_termination_reason = "not_run";
      std::string normalization_mode = "none";
      double normalization_reference_lambda = 1.0e-5;
      double normalization_scale = 1.0;
      double normalization_budget_fraction = 0.30;
      double normalization_base_improvement_budget = 0.0;
      double normalization_reference_displacement_m = 0.025;
      double pre_support_coverage = std::numeric_limits<double>::quiet_NaN();
      double post_support_coverage = std::numeric_limits<double>::quiet_NaN();
      int support_sample_count = 0;
      int pre_support_valid_count = 0;
      int post_support_valid_count = 0;
      bool support_full_valid = false;
      bool optimization_success = false;
      bool objective_allowed = false;
      bool objective_applied = false;
      double selection_score = std::numeric_limits<double>::quiet_NaN();
      std::string selection_reason = "not_selected";
      int candidate_rank = 0;
      bool p1_descent = false;
      bool rank_eligible = false;
      bool replacement_accepted = false;
      std::string replacement_reason = "not_evaluated";
      bool incumbent_available = false;
      // Incumbent presence is an identity fact, not a statement that a
      // candidate-specific fixed-200 comparison obtained full support.
      // Rejected-candidate evidence must retain that identity even when the
      // comparison itself fails closed.
      void markIncumbentAvailable() { incumbent_available = true; }
      // Finite by construction: availability disambiguates the startup case
      // without weakening the candidate CSV finite-value contract.
      double incumbent_mean_c_pi = 0.0;
      double incumbent_max_c_pi = 0.0;
      std::string replacement_comparison_mode = "full_profile";
      double replacement_comparison_duration_s = 0.0;
      double replacement_candidate_mean_c_pi = 0.0;
      double replacement_candidate_max_c_pi = 0.0;
      std::string fallback_reason = "not_evaluated";
      std::string support_signature;
      std::string initial_control_points_hash;
      std::string final_control_points_hash;
      std::string p1_config_hash;
      uint64_t planning_attempt_id = 0;
      uint64_t candidate_id = 0;
      uint64_t snapshot_generation_id = 0;
      double query_base_time_s = std::numeric_limits<double>::quiet_NaN();
      bool selected = false;
      int solver_result = 0;
      std::string termination_reason;
      int iteration_count = 0;
      std::string aggregation_mode = "adaptive_mean";
      double aggregation_temperature = 0.0;
      double aggregation_tail_fraction = 0.0;
      int adaptive_sample_count = 0;
      int fixed_sample_count = 200;
      double peak_contribution = 0.0;
      P1FanoutDiagnostics fanout;
    };

    struct P1IntegrityVizSample
    {
      Eigen::Vector3d position = Eigen::Vector3d::Zero();
      Eigen::Vector3d grad = Eigen::Vector3d::Zero();
      Eigen::Vector3d push = Eigen::Vector3d::Zero();
      double cost = std::numeric_limits<double>::quiet_NaN();
      double t_s = 0.0;
      bool hit = false;
      bool stale = false;
      bool unknown = false;
      std::string reason = "not_evaluated";
    };

    struct P4GuideViz
    {
      std::vector<Eigen::Vector3d> original_path;
      std::vector<Eigen::Vector3d> risk_path;
      std::vector<Eigen::Vector3d> selected_path;
      Eigen::Vector3d segment_start = Eigen::Vector3d::Zero();
      Eigen::Vector3d segment_end = Eigen::Vector3d::Zero();
      P4AStarMetrics metrics;
      bool risk_selected = false;
    };

    BsplineOptimizer() {}
    ~BsplineOptimizer() {}

    /* main API */
    void setEnvironment(const GridMap::Ptr &map);
    void setEnvironment(const GridMap::Ptr &map, const fast_planner::ObjPredictor::Ptr mov_obj);
    void setParam(rclcpp::Node::SharedPtr node);
    Eigen::MatrixXd BsplineOptimizeTraj(const Eigen::MatrixXd &points, const double &ts,
                                        const int &cost_function, int max_num_id, int max_time_id);

    /* helper function */

    // required inputs
    void setControlPoints(const Eigen::MatrixXd &points);
    void setBsplineInterval(const double &ts);
    void setSwarmTrajs(SwarmTrajData *swarm_trajs_ptr);
    void setDroneId(const int drone_id);
    void setRiskSnapshot(std::shared_ptr<const iap::RiskGridSnapshot> snapshot,
                         double query_base_time_s);
    void setP1PlanningRiskContext(P1PlanningRiskContext context);
    void clearRiskSnapshot();
    void setP4RiskSnapshot(std::shared_ptr<const iap::RiskGridSnapshot> snapshot,
                           double query_base_time_s);
    void clearP4RiskSnapshot();

    // optional inputs
    void setGuidePath(const vector<Eigen::Vector3d> &guide_pt);
    void setWaypoints(const vector<Eigen::Vector3d> &waypts,
                      const vector<int> &waypt_idx); // N-2 constraints at most
    void setLocalTargetPt(const Eigen::Vector3d local_target_pt) { local_target_pt_ = local_target_pt; };

    void optimize();

    ControlPoints getControlPoints() { return cps_; };

    AStar::Ptr a_star_;
    std::vector<Eigen::Vector3d> ref_pts_;

    std::vector<ControlPoints> distinctiveTrajs(vector<std::pair<int, int>> segments);
    const P1FanoutDiagnostics &lastP1FanoutDiagnostics() const {
      return last_p1_fanout_diagnostics_;
    }
    std::vector<ControlPoints> supplementP1RiskGradientCandidates(
        const ControlPoints &base,
        const std::shared_ptr<const iap::RiskGridSnapshot> &snapshot,
        double query_base_time_s, int remaining_capacity);
    std::vector<std::pair<int, int>> initControlPoints(Eigen::MatrixXd &init_points, bool flag_first_init = true);
    bool BsplineOptimizeTrajRebound(Eigen::MatrixXd &optimal_points, double ts); // must be called after initControlPoints()
    bool BsplineOptimizeTrajRebound(Eigen::MatrixXd &optimal_points, double &final_cost, const ControlPoints &control_points, double ts);
    bool BsplineOptimizeTrajBasePrepass(
        Eigen::MatrixXd &optimal_points, double &final_cost,
        const ControlPoints &control_points, double ts);
    bool BsplineOptimizeTrajNormalizedP1(
        Eigen::MatrixXd &optimal_points, double &final_cost,
        const ControlPoints &seed_control_points, double ts,
        const P1BasePrepassTrace &base_prepass, std::string *reason = nullptr);
    bool BsplineOptimizeTrajRefine(const Eigen::MatrixXd &init_points, const double ts, Eigen::MatrixXd &optimal_points);

    inline int getOrder(void) { return order_; }
    inline double getSwarmClearance(void) { return swarm_clearance_; }
    const P1IntegrityMetrics &getLastP1IntegrityMetrics() const { return last_p1_metrics_; }
    const std::vector<P1IntegrityVizSample> &getLastP1IntegrityVizSamples() const { return last_p1_viz_samples_; }
    const P1IntegrityConfig &getP1IntegrityConfig() const { return p1_config_; }
    std::string p1AcceptedTrajectoryRiskProfilePath() const;
    std::string p1AcceptedTrajectoryRiskProfileContextPath() const;
    const P4RiskAStarConfig &getP4RiskAStarConfig() const { return p4_config_; }
    const std::vector<P4GuideViz> &getLastP4GuideViz() const { return last_p4_guides_; }
    const OptimizerCostBreakdown &getLastOptimizerCostBreakdown() const { return last_optimizer_cost_breakdown_; }
    const P1OptimizationTrace &getLastP1OptimizationTrace() const { return last_p1_optimization_trace_; }
    const P1BasePrepassTrace &getLastP1BasePrepassTrace() const {
      return last_p1_base_prepass_trace_;
    }
    std::string p1CandidateOptimizationPath() const;
    std::string p1ReplacementDecisionPath() const;
    std::string p1CandidateRetainedProfilePath() const;
    std::string p1CandidateControlPointsPath() const;
    std::string p1CandidateProfilePath() const;
    std::string p1PrequalificationCandidateProfilePath() const;
    std::string p1CandidatePairwisePath() const;
    std::string p1OptimizerCheckpointPath() const;
    std::string p0OccupancyQueryEvidencePath() const;
    void setLastP1OptimizationSelected(bool selected);
    void writeP1OptimizationTrace(const P1OptimizationTrace &trace) const;
    bool writeP1PrequalificationCandidateProfile(
        UniformBspline candidate, bool selected,
        const std::string &phase = "final");
    void writeP1ReplacementDecision(const P1OptimizationTrace &trace,
                                    uint64_t incumbent_trajectory_id,
                                    double incumbent_start_stamp_s,
                                    const std::string &final_trajectory_source,
                                    const std::string &publish_identity) const;
    bool writeP1CandidateRetainedProfile(
        UniformBspline candidate, uint64_t planning_attempt_id,
        uint64_t candidate_id,
        const UniformBspline *incumbent, uint64_t incumbent_trajectory_id,
        const std::string &final_trajectory_source,
        double incumbent_start_t_s = 0.0) const;
    void setP1IntegrityConfigForTest(const P1IntegrityConfig &config) { p1_config_ = config; }
    void setP4RiskAStarConfigForTest(const P4RiskAStarConfig &config) { p4_config_ = config; }
    bool evaluateReboundCostForTest(const Eigen::MatrixXd &control_points, double ts,
                                    double &cost, Eigen::MatrixXd &gradient);
    bool evaluateP1RawCostForTest(const Eigen::MatrixXd &control_points, double ts,
                                  double &cost, Eigen::MatrixXd &gradient);
    bool optimizeReboundCostForTest(Eigen::MatrixXd &control_points, double ts,
                                    int max_iterations, double &final_cost,
                                    int &iterations);
    bool optimizeP1BasePrepassForTest(Eigen::MatrixXd &control_points, double ts,
                                      int max_iterations, double &final_cost,
                                      int &iterations);
    bool prepareP1NormalizedStage(const Eigen::MatrixXd &seed_control_points,
                                  double ts,
                                  const P1BasePrepassTrace &base_prepass,
                                  std::string *reason = nullptr);
    void clearP1NormalizedStage();
    double p1LbfgsGradientEpsilon(double initial_weighted_p1_gradient_norm,
                                  double variable_norm) const;
    void setP1PreOptimizationTrajectoryForTest(
        const Eigen::MatrixXd &control_points, double interval_s);
    bool writeP1AcceptedTrajectoryRiskProfile(UniformBspline trajectory,
                                              uint64_t profile_seq,
                                              uint64_t trajectory_id,
                                              double stamp_s,
                                              double planning_start_s =
                                                  std::numeric_limits<double>::quiet_NaN(),
                                              const std::string &trajectory_frame_id = "map",
                                              double trajectory_start_stamp_s =
                                                  std::numeric_limits<double>::quiet_NaN(),
                                              double trajectory_start_t_s = 0.0,
                                              double window_duration_s =
                                                  std::numeric_limits<double>::infinity()) const;
    iap::P1AcceptedContextValidation validateP1AcceptedTrajectoryRiskContext(
        UniformBspline trajectory, double accepted_stamp_s,
        const std::string &trajectory_frame_id) const;
    P1FixedLatticeRiskSummary evaluateP1FixedLatticeRisk(
        UniformBspline trajectory,
        double trajectory_start_t_s = 0.0,
        double window_duration_s = std::numeric_limits<double>::infinity()) const;

  private:
    GridMap::Ptr grid_map_;
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
    double bspline_interval_; // B-spline knot span
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

    P1IntegrityConfig p1_config_;
    P4RiskAStarConfig p4_config_;
    P1IntegrityMetrics last_p1_metrics_;
    std::vector<P1IntegrityVizSample> last_p1_viz_samples_;
    std::vector<P4GuideViz> last_p4_guides_;
    OptimizerCostBreakdown last_optimizer_cost_breakdown_;
    P1OptimizationTrace last_p1_optimization_trace_;
    P1BasePrepassTrace last_p1_base_prepass_trace_;
    P1FanoutDiagnostics last_p1_fanout_diagnostics_;
    double last_rebound_total_gradient_norm_{std::numeric_limits<double>::quiet_NaN()};
    bool suppress_rebound_collision_for_test_{false};
    bool p1_base_prepass_active_{false};
    struct P1NormalizedStage
    {
      bool enabled = false;
      double reference_lambda = 1.0e-5;
      double budget_fraction = 0.30;
      double reference_displacement_m = 0.025;
      double base_improvement_budget = 0.0;
      double raw_gradient_norm = 0.0;
      double scale = 1.0;
      double seed_raw_cost = 0.0;
      Eigen::MatrixXd seed_control_points;
      P1BasePrepassTrace base_prepass;
    } p1_normalized_stage_;
    std::shared_ptr<const iap::RiskGridSnapshot> risk_snapshot_;
    double risk_query_base_time_s_{0.0};
    P1PlanningRiskContext p1_risk_context_;
    bool p1_debug_csv_header_written_{false};
    struct P1PreOptimizationTrace
    {
      Eigen::MatrixXd control_points;
      double interval_s = std::numeric_limits<double>::quiet_NaN();
    };
    std::map<std::pair<uint64_t, uint64_t>, P1PreOptimizationTrace>
        p1_pre_optimization_traces_;
    struct P1CandidateArtifact
    {
      Eigen::MatrixXd initial_control_points;
      Eigen::MatrixXd final_control_points;
      double interval_s = std::numeric_limits<double>::quiet_NaN();
      std::shared_ptr<const iap::RiskGridSnapshot> snapshot;
      double query_base_time_s = std::numeric_limits<double>::quiet_NaN();
      std::vector<P1OptimizerCheckpoint> checkpoints;
    };
    std::map<std::pair<uint64_t, uint64_t>, P1CandidateArtifact>
        p1_candidate_artifacts_;
    std::vector<P1OptimizerCheckpoint> current_p1_checkpoints_;
    Eigen::VectorXd current_p1_last_accepted_x_;
    int current_p1_restart_index_{0};

    int variable_num_;              // optimization variables
    int iter_num_;                  // iteration of the solver
    Eigen::VectorXd best_variable_; //
    double min_cost_;               //

    Eigen::Vector3d local_target_pt_; 

#define INIT_min_ellip_dist_ 123456789.0123456789
    double min_ellip_dist_;

    ControlPoints cps_;

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
    void calcFitnessCost(const Eigen::MatrixXd &q, double &cost, Eigen::MatrixXd &gradient);
    void calcIntegrityTrajectoryCost(const Eigen::MatrixXd &q, double &cost,
                                     Eigen::MatrixXd &gradient,
                                     P1IntegrityMetrics &metrics);
    bool cubicBasisForTime(double t, int control_point_count,
                           int &first_control_point,
                           double weights[4]) const;
    void writeP1DebugCsv(const P1IntegrityMetrics &metrics) const;
    void writeP1CandidateOptimizationCsv(const P1OptimizationTrace &trace) const;
    void captureP1PreOptimizationTrajectory(
        const Eigen::MatrixXd &control_points, double interval_s);
    void captureP1PostOptimizationTrajectory(
        const Eigen::MatrixXd &control_points, double interval_s);
    void writeP1CandidateSidecars(const P1OptimizationTrace &trace) const;
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
