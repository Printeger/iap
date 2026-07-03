#ifndef _PLANNER_MANAGER_H_
#define _PLANNER_MANAGER_H_

#include <stdlib.h>

#include <cstdint>
#include <memory>

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
      uint64_t generation_id = 0;
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

    void deliverTrajToOptimizer(void) { bspline_optimizer_->setSwarmTrajs(&swarm_trajs_buf_); };

    void setDroneIdtoOpt(void) { bspline_optimizer_->setDroneId(pp_.drone_id); }

    double getSwarmClearance(void) { return bspline_optimizer_->getSwarmClearance(); }

    bool checkCollision(int drone_id);
    std::shared_ptr<const iap::RiskGridSnapshot> acquireRiskGridSnapshot() const;
    const PlanningRiskContext &beginPlanningRiskContext(double now_s);
    void clearPlanningRiskContext();
    const PlanningRiskContext &planningRiskContext() const { return planning_risk_context_; }
    std::shared_ptr<const iap::RiskGridSnapshot> currentPlanningRiskSnapshot() const { return planning_risk_context_.snapshot; }
    double currentPlanningQueryBaseTime() const { return planning_risk_context_.query_base_time_s; }
    uint64_t currentPlanningGenerationId() const { return planning_risk_context_.generation_id; }
    void setPlanningRiskContextForTest(std::shared_ptr<const iap::RiskGridSnapshot> snapshot,
                                       double query_base_time_s);

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
    uint64_t p2_batch_id_{0};
    uint64_t p3_batch_id_{0};
    PlanningRiskContext planning_risk_context_;

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
