#ifndef _PLANNER_MANAGER_H_
#define _PLANNER_MANAGER_H_

#include <stdlib.h>

#include <cstdint>
#include <functional>
#include <memory>
#include <string>

#include <bspline_opt/bspline_optimizer.h>
#include <bspline_opt/uniform_bspline.h>
#include <ego_planner/p2_candidate_ranking.h>
#include <ego_planner/p3_reference_bias.h>
#include <traj_utils/msg/data_disp.hpp>
#include <plan_env/grid_map.h>
#include <plan_env/obj_predictor.h>
#include <traj_utils/plan_container.hpp>
#include <rclcpp/rclcpp.hpp>
#include <traj_utils/planning_visualization.h>

namespace ego_planner
{
  class P0RiskGridRuntime;
  class P5RuntimeIntegrityGate;
  class SafetyRvizPublisher;
}

namespace iap
{
  class RiskGridSnapshot;
}

namespace ego_planner
{

  // Fast Planner Manager
  // Key algorithms of mapping and planning are called

  class EGOPlannerManager
  {
    // SECTION stable
  public:
    struct PlanningRiskContext
    {
      std::shared_ptr<const iap::RiskGridSnapshot> snapshot;
      double query_base_time_s = 0.0;
      double planning_start_s = 0.0;
      double snapshot_acquired_s = 0.0;
      double snapshot_stamp_s = 0.0;
      double optimizer_start_s = 0.0;
      double optimizer_end_s = 0.0;
      double accepted_s = 0.0;
      double pre_publish_s = 0.0;
      double publish_s = 0.0;
      uint64_t generation_id = 0;
      uint64_t planning_attempt_id = 0;
      uint64_t candidate_id = 0;
      bool p1_objective_allowed = true;
      bool p1_objective_applied = false;
      std::string p1_fallback_reason = "none";
      bool active = false;
    };

    EGOPlannerManager();
    ~EGOPlannerManager();

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /* main planning interface */
    bool reboundReplan(Eigen::Vector3d start_pt, Eigen::Vector3d start_vel, Eigen::Vector3d start_acc,
                       Eigen::Vector3d end_pt, Eigen::Vector3d end_vel, bool flag_polyInit, bool flag_randomPolyTraj);
    bool EmergencyStop(Eigen::Vector3d stop_pos);
    bool planGlobalTraj(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
                        const Eigen::Vector3d &end_pos, const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc);
    bool planGlobalTrajWithP3ReferenceBias(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
                                           const Eigen::Vector3d &end_pos, const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc);
    bool planGlobalTrajWaypoints(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
                                 const std::vector<Eigen::Vector3d> &waypoints, const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc);
    bool applyLocalTargetP3ReferenceBias(const Eigen::Vector3d &start_pt, const Eigen::Vector3d &end_pt,
                                         Eigen::Vector3d &local_target_pt, Eigen::Vector3d &local_target_vel);

    void initPlanModules(rclcpp::Node::SharedPtr &node, PlanningVisualization::Ptr vis = NULL);
    using TimeProvider = std::function<rclcpp::Time()>;
    void setTimeProvider(TimeProvider provider);
    rclcpp::Time plannerNow() const;

    void deliverTrajToOptimizer(void) { bspline_optimizer_->setSwarmTrajs(&swarm_trajs_buf_); };

    void setDroneIdtoOpt(void) { bspline_optimizer_->setDroneId(pp_.drone_id); }

    double getSwarmClearance(void) { return bspline_optimizer_->getSwarmClearance(); }

    bool checkCollision(int drone_id);
    std::shared_ptr<const iap::RiskGridSnapshot> acquireRiskGridSnapshot() const;
    const PlanningRiskContext &beginPlanningRiskContext(double now_s);
    const PlanningRiskContext &beginPlanningRiskContextWithSnapshot(
        double now_s,
        std::shared_ptr<const iap::RiskGridSnapshot> snapshot,
        uint64_t planning_attempt_id = 0);
    void clearPlanningRiskContext();
    const PlanningRiskContext &planningRiskContext() const { return planning_risk_context_; }
    std::shared_ptr<const iap::RiskGridSnapshot> currentPlanningRiskSnapshot() const { return planning_risk_context_.snapshot; }
    double currentPlanningQueryBaseTime() const { return planning_risk_context_.query_base_time_s; }
    uint64_t currentPlanningGenerationId() const { return planning_risk_context_.generation_id; }
    void setPlanningRiskContextForTest(std::shared_ptr<const iap::RiskGridSnapshot> snapshot,
                                       double query_base_time_s);
    // P1 candidates are fail-closed against the same immutable snapshot they
    // were optimized with. These methods are intentionally separate from P5.
    bool planningRiskContextFresh(double now_s, std::string *reason = nullptr) const;
    bool preparePlanningRiskPublish(double now_s, std::string *reason = nullptr);
    bool finalizeP1AcceptedRiskProfile(double publish_stamp_s);
    bool recordP1MetricsOnlyReferenceObservation(double observation_stamp_s);
    std::string p1PlanningContextTimelinePath() const;
    bool p1AdmissionEnabled() const;
    const std::string &lastP1RejectionReason() const { return last_p1_rejection_reason_; }
    bool lastP1RejectionRequiresNewGeneration() const {
      return last_p1_rejection_requires_new_generation_;
    }
    void recordP1RetryDeferred(
        const std::string &reason, double stamp_s,
        std::shared_ptr<const iap::RiskGridSnapshot> snapshot);
    void recordP1StaleRejection(const std::string &reason, double stamp_s);

    PlanParameters pp_;
    LocalTrajData local_data_;
    GlobalTrajData global_data_;
    GridMap::Ptr grid_map_;
    fast_planner::ObjPredictor::Ptr obj_predictor_;    
    SwarmTrajData swarm_trajs_buf_;
    std::unique_ptr<P0RiskGridRuntime> p0_risk_grid_runtime_;
    std::unique_ptr<P5RuntimeIntegrityGate> p5_integrity_gate_;
    std::shared_ptr<SafetyRvizPublisher> safety_viz_;
    P2CandidateRankingConfig p2_config_;
    P3ReferenceBiasConfig p3_config_;

  private:
    /* main planning algorithms & modules */
    PlanningVisualization::Ptr visualization_;

    // ros::Publisher obj_pub_; //zx-todo 

    BsplineOptimizer::Ptr bspline_optimizer_;

    int continous_failures_count_{0};
    uint64_t p1_accepted_profile_seq_{0};
    uint64_t p1_metrics_reference_observed_trajectory_id_{0};
    uint64_t p1_planning_attempt_seq_{0};
    bool p1_activation_recorded_{false};
    bool has_p1_preference_incumbent_{false};
    uint64_t p2_batch_id_{0};
    uint64_t p3_batch_id_{0};
    PlanningRiskContext planning_risk_context_;
    TimeProvider time_provider_;
    std::string trajectory_frame_id_{"map"};
    std::string last_p1_rejection_reason_;
    bool last_p1_rejection_requires_new_generation_{false};

    void appendPlanningRiskContextTimeline(const std::string &stage,
                                           double stamp_s,
                                           const std::string &outcome,
                                           const std::string &reason,
                                           const std::string &fallback_branch = "",
                                           const PlanningRiskContext *context_override = nullptr) const;
    std::string p1PreAdmissionAttemptPath() const;
    void writeP1PreAdmissionAttempt(
        const std::string &stage, uint64_t candidate_id,
        const UniformBspline &initial_trajectory,
        const iap::P1AcceptedContextValidation &initial_validation,
        const UniformBspline *base_optimized_trajectory,
        bool base_optimizer_success, const std::string &base_reason,
        const std::string &p1_admission_verdict,
        const std::string &p1_admission_reason) const;

    void updateTrajInfo(const UniformBspline &position_traj, const rclcpp::Time time_now);

    void reparamBspline(UniformBspline &bspline, vector<Eigen::Vector3d> &start_end_derivative, double ratio, Eigen::MatrixXd &ctrl_pts, double &dt,
                        double &time_inc);

    bool refineTrajAlgo(UniformBspline &traj, vector<Eigen::Vector3d> &start_end_derivative, double ratio, double &ts, Eigen::MatrixXd &optimal_control_points);

    // !SECTION stable

    // SECTION developing

  public:
    typedef unique_ptr<EGOPlannerManager> Ptr;

    // !SECTION
  };
} // namespace ego_planner

#endif
