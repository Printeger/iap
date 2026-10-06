#ifndef _PLANNER_MANAGER_H_
#define _PLANNER_MANAGER_H_

#include <fstream>
#include <ego_planner/prediction_input.h>
#include <iap/srv/get_grid_map_prediction_input.hpp>

#include <stdlib.h>
#include <deque>
#include <limits>
#include <optional>
#include <unordered_map>
#include <unordered_set>
#include <gnss_comm/gnss_ros.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <iap/msg/integrity_report.hpp>
#include <iap/predictor/predictor_module.hpp>

#include <bspline_opt/bspline_optimizer.h>
#include <bspline_opt/uniform_bspline.h>
#include <traj_utils/msg/data_disp.hpp>
#include <plan_env/grid_map.h>
#include <plan_env/obj_predictor.h>
#include <traj_utils/plan_container.hpp>
#include <rclcpp/rclcpp.hpp>
#include <traj_utils/planning_visualization.h>

namespace ego_planner
{

  // Fast Planner Manager
  // Key algorithms of mapping and planning are called

  class EGOPlannerManager
  {
    friend struct EGOPlannerManagerTestAccess;
    // SECTION stable
  public:
    EGOPlannerManager();
    ~EGOPlannerManager();

    struct PlanningTimings {
      double freeze_s = 0.0;
      double prediction_preparation_s = 0.0;
      double searcher_initialization_s = 0.0; // Startup allocation, not per round.
      double backend_s = 0.0;
      double final_checks_s = 0.0;
    };
    const PlanningTimings& planningTimings() const { return planning_timings_; }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /* main planning interface */
    bool reboundReplan(Eigen::Vector3d start_pt, Eigen::Vector3d start_vel, Eigen::Vector3d start_acc,
                       Eigen::Vector3d end_pt, Eigen::Vector3d end_vel, bool flag_polyInit, bool flag_randomPolyTraj);
    // Freeze one spatial prediction round. The returned version is required
    // by GridMap::queryRisk; binding failure is represented by query status.
    uint64_t bindRiskPrediction(const iap::IntegritySnapshot& snapshot,
                                double reference_time_s, std::shared_ptr<const FrozenOccupancyEpoch> occupancy = {});
    bool EmergencyStop(Eigen::Vector3d stop_pos);
    bool planCheckedBrake(const Eigen::Vector3d& position,
                          const Eigen::Vector3d& velocity,
                          const Eigen::Vector3d& acceleration);
    bool planGlobalTraj(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
                        const Eigen::Vector3d &end_pos, const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc);
    bool planGlobalTrajWaypoints(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
                                 const std::vector<Eigen::Vector3d> &waypoints, const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc);

    void initPlanModules(rclcpp::Node::SharedPtr &node, PlanningVisualization::Ptr vis = NULL);

    void deliverTrajToOptimizer(void) { bspline_optimizer_->setSwarmTrajs(&swarm_trajs_buf_); };

    void setDroneIdtoOpt(void) { bspline_optimizer_->setDroneId(pp_.drone_id); }

    double getSwarmClearance(void) { return bspline_optimizer_->getSwarmClearance(); }

    bool checkCollision(int drone_id);

    struct TrajectoryAssessment {
      int trajectory_id = -1; // Owning curve for two-segment failure evidence.
      bool budget_exhausted = false;
      GridExecutionReason execution_reason = GridExecutionReason::OK;
      double first_execution_time_s = std::numeric_limits<double>::quiet_NaN();
      Eigen::Vector3d first_execution_position = Eigen::Vector3d::Constant(
          std::numeric_limits<double>::quiet_NaN());
      GridPlanningCell first_execution_cell;
      // Independent of the first physical/motion rejection on the curve.
      double first_unobserved_time_s = std::numeric_limits<double>::quiet_NaN();
      Eigen::Vector3d first_unobserved_position = Eigen::Vector3d::Constant(
          std::numeric_limits<double>::quiet_NaN());
      GridPlanningCell first_unobserved_cell;
      double checked_from_time_s = 0.0;
      double checked_to_time_s = 0.0;
      double sample_step_s = 0.0;
      double evaluation_time_s = 0.0;
      GridMotionContext evaluated_motion;
      std::shared_ptr<const GridMapFailureSnapshot> failure_snapshot;
      double first_advisory_time_s = std::numeric_limits<double>::quiet_NaN();
      size_t advisory_avoid_samples = 0;
      size_t advisory_unknown_samples = 0;
      size_t sampled_points = 0;
      bool map_changed = false;
      std::shared_ptr<const FrozenOccupancyEpoch> physical_epoch;
      uint64_t evaluated_generation = 0;
      uint8_t evaluated_motion_quality = 0;
      double evaluated_motion_error_proxy_m =
          std::numeric_limits<double>::quiet_NaN();
      bool executable() const { return !budget_exhausted && execution_reason == GridExecutionReason::OK; }
    };
    GridMotionContext currentMotionContext(bool allow_bridged = false) const;
    TrajectoryAssessment assessTrajectory(const UniformBspline& trajectory,
                                          uint64_t risk_version, double now_s,
                                          bool allow_bridged = false,
                                          double from_time_s = 0.0,
                                          double to_time_s =
                                              std::numeric_limits<double>::infinity(),
                                          const GridPlanningContext* physical_context = nullptr,
                                          bool check_connection = true);
    TrajectoryAssessment assessRemainingTrajectory(double now_s);
    GridPlanningCell queryLocalTargetCell(const Eigen::Vector3d& position,
                                          double now_s) const;
    PlanningBudget::Ptr planningBudget() const { return planning_budget_; }
    bool beginPlanningView(double budget_seconds = 1.5);
    bool hasPlanningView() const { return planning_view_.has_value(); }
    void setPlanningConnection(rclcpp::Time start_time, int predecessor_id);
    bool hasPendingTrajectory() const { return pending_trajectory_.has_value(); }
    const LocalTrajData& publicationTrajectory() const {
      return pending_trajectory_ ? *pending_trajectory_ : local_data_;
    }
    void observeExecutingTrajectory(int trajectory_id);
    bool publicationStillTimely() const;
    void discardUnpublishedTrajectory(const LocalTrajData& predecessor);
    enum class PlanFailure { None, Budget, Target, Search, Curve, Release, Connection };
    PlanFailure lastPlanFailure() const { return last_plan_failure_; }
    void setLocalTargets(std::vector<LocalTarget> targets) { planning_targets_ = std::move(targets); }
    double terminalSpeedLimit(const Eigen::Vector3d& position,
                              const Eigen::Vector3d& reference_velocity) const;
    void endPlanningView();
    std::optional<Eigen::Vector3d> planningReferencePosition() const {
      return planning_view_ ? planning_view_->reference_position : std::nullopt;
    }
    void setLatestOdometryProvider(std::function<
        nav_msgs::msg::Odometry::ConstSharedPtr()> provider) {
      latest_odom_provider_ = std::move(provider);
    }
    GridPlanningRisk queryPlanningViewAdvisory(const Eigen::Vector3d& position) const;
    GridPlanningCell queryPlanningViewCell(const Eigen::Vector3d& position) const;
    std::optional<uint64_t> planningEvidenceFingerprint(
        const Eigen::Vector3d& start, const Eigen::Vector3d& target) const;
    void capturePlanningStall(const Eigen::Vector3d& start,
                              const Eigen::Vector3d& target);
    void captureRemainingFailure(const std::string& kind,
        const Eigen::Vector3d& expected, const Eigen::Vector3d& actual,
        double error_m, int trajectory_id, double command_time_s,
        double odom_age_s, double map_age_s,
        GridExecutionReason reason,
        const TrajectoryAssessment* assessment = nullptr);
    AStar::Failure lastSearchFailure() const {
      return bspline_optimizer_->a_star_->lastResult().failure;
    }


    PlanParameters pp_;
    LocalTrajData local_data_;
    GlobalTrajData global_data_;
    GridMap::Ptr grid_map_;
    fast_planner::ObjPredictor::Ptr obj_predictor_;
    SwarmTrajData swarm_trajs_buf_;

  private:
    // Input callbacks and planning run on the original serial executor.
    void initRiskInputs(const rclcpp::Node::SharedPtr& node);
    uint64_t beginRiskQuery();
    void rangeCallback(const gnss_comm::msg::GnssMeasMsg::ConstSharedPtr msg);
    rclcpp::Node::SharedPtr node_;
    iap::PredictorParams predictor_params_;
    double risk_validity_s_ = 0.5;
    GridPlanningRiskPolicy planning_risk_policy_;
    double motion_body_radius_m_ = 0.35;
    double motion_tracking_reserve_m_ = 0.10;
    double motion_start_tolerance_m_ = 0.30;
    double motion_budget_m_ = 0.55;
    double motion_max_age_s_ = 0.5;
    double environment_max_age_s_ = 0.5;
    bool capture_failure_map_ = false;
    bool search_performance_diagnostics_ = false;
    PlanningTimings planning_timings_;
    std::unordered_set<std::string> captured_failure_kinds_;
    uint64_t planning_risk_version_ = 0;
    double planning_time_s_ = 0.0;
    GridMotionContext planning_motion_;
    struct PlanningView {
      std::shared_ptr<const FrozenOccupancyEpoch> physical;
      std::shared_ptr<const GridMapFailureSnapshot> snapshot;
      uint64_t risk_version = 0;
      uint64_t generation = 0;
      double time_s = 0.0;
      GridMotionContext motion;
      std::optional<Eigen::Vector3d> reference_position;
      GridPlanningContext physical_context;
      std::function<GridPlanningRisk(const Eigen::Vector3d&)> advisory_query;
      double advisory_valid_until_s = 0.0;
      mutable GridPlanningQueryStats advisory_stats;
    };
    std::optional<PlanningView> planning_view_;
    PlanningBudget::Ptr planning_budget_;
    std::vector<LocalTarget> planning_targets_;
    std::optional<LocalTrajData> pending_trajectory_;
    std::optional<rclcpp::Time> connection_time_;
    int connection_predecessor_ = -1;
    int next_trajectory_id_ = 0;
    PlanFailure last_plan_failure_ = PlanFailure::None;
    TrajectoryAssessment last_candidate_assessment_;
    std::function<nav_msgs::msg::Odometry::ConstSharedPtr()>
        latest_odom_provider_;
    void captureFailureMap(const std::string& kind, const Eigen::Vector3d& point,
                           const Eigen::Vector3d& other,
                           const GridPlanningCell& cell,
                           const AStar::Result* search = nullptr,
                           const BsplineOptimizer::SearchFailureContext* context = nullptr,
                           const UniformBspline* trajectory = nullptr,
                           const TrajectoryAssessment* assessment = nullptr);
    double last_runtime_advisory_query_s_ =
        -std::numeric_limits<double>::infinity();
    iap::CurrentIntegrityState current_integrity_;
    std::shared_ptr<const iap::msg::IntegrityReport> pending_integrity_;
    rclcpp::CallbackGroup::SharedPtr integrity_callback_group_;
    nav_msgs::msg::Odometry::ConstSharedPtr risk_odom_;
    bool risk_frame_valid_ = false;
    bool origin_set_ = false;
    Eigen::Vector3d origin_ecef_ = Eigen::Vector3d::Zero();
    std::vector<double> iono_params_;
    std::unordered_map<uint32_t, gnss_comm::EphemPtr> ephem_cache_;
    std::unordered_map<uint32_t, gnss_comm::GloEphemPtr> glo_ephem_cache_;
    std::deque<iap::GnssEpoch> epochs_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr risk_odom_sub_;
    rclcpp::Subscription<iap::msg::IntegrityReport>::SharedPtr integrity_sub_;
    rclcpp::Subscription<gnss_comm::msg::GnssMeasMsg>::SharedPtr range_sub_;
    rclcpp::Subscription<gnss_comm::msg::GnssEphemMsg>::SharedPtr ephem_sub_;
    rclcpp::Subscription<gnss_comm::msg::GnssGloEphemMsg>::SharedPtr glo_ephem_sub_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr receiver_lla_sub_;
    rclcpp::Subscription<gnss_comm::msg::GnssIonosphereParameter>::SharedPtr iono_sub_;
    std::ofstream planning_metrics_, export_metrics_;
    uint64_t planning_calls_at_start_=0;
    mutable std::mutex epochs_mutex_;
    rclcpp::CallbackGroup::SharedPtr export_callback_group_;
    rclcpp::Service<iap::srv::GetGridMapPredictionInput>::SharedPtr prediction_export_service_;
    std::shared_ptr<std::atomic<uint64_t>> predictor_calls_ = std::make_shared<std::atomic<uint64_t>>(0);
    iap::IntegritySnapshot capturePredictionSnapshot(double now) const;
    void initPredictionExport();

    /* main planning algorithms & modules */
    PlanningVisualization::Ptr visualization_;

    // ros::Publisher obj_pub_; //zx-todo

    BsplineOptimizer::Ptr bspline_optimizer_;

    int continous_failures_count_{0};

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
