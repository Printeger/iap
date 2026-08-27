#ifndef _P5_RUNTIME_INTEGRITY_GATE_H_
#define _P5_RUNTIME_INTEGRITY_GATE_H_

#include <functional>
#include <memory>
#include <mutex>
#include <limits>
#include <string>
#include <vector>

#include <iap/msg/integrity_report.hpp>
#include <iap/planner/risk_grid_map.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <ego_planner/safety_rviz_publisher.h>
#include <traj_utils/plan_container.hpp>

namespace ego_planner {

enum class P5GateAction {
  OK = 0,
  REQUEST_REPLAN = 1,
  REQUEST_EMERGENCY_STOP_CANDIDATE = 2,
};

enum class P5GateReason {
  DISABLED,
  OK,
  CURRENT_INVALID,
  CURRENT_STALE,
  CURRENT_LOW_MARGIN,
  FUTURE_BAD,
  FUTURE_UNKNOWN,
  AL_INVALID,
  FINAL_GATE_FAILED,
  SNAPSHOT_UNAVAILABLE,
};

enum class PredAlertLimitMode {
  CURRENT_MSG_CONSTANT,
  CONFIG_CONSTANT,
  OCCUPANCY_CLEARANCE,
  VERTICAL_BOUND_ONLY,
};

struct PredAlertLimitSample {
  bool valid = false;
  double hal = std::numeric_limits<double>::quiet_NaN();
  double val = std::numeric_limits<double>::quiet_NaN();
  std::string reason = "not_evaluated";
};

class PredAlertLimitProvider {
 public:
  struct Config {
    PredAlertLimitMode mode = PredAlertLimitMode::CURRENT_MSG_CONSTANT;
    double constant_hal_m = 10.0;
    double constant_val_m = 10.0;
    double min_hal_m = 0.1;
    double max_hal_m = 50.0;
    double min_val_m = 0.1;
    double max_val_m = 50.0;
    double clearance_search_radius_m = 5.0;
    double clearance_step_m = 0.25;
    double drone_radius_m = 0.35;
    double clearance_scale = 1.0;
    double vertical_scale = 1.0;
  };

  using OccupancyQuery = std::function<bool(const Eigen::Vector3d&)>;
  using MapRegionQuery =
      std::function<bool(Eigen::Vector3d* origin, Eigen::Vector3d* size)>;
  using ResolutionQuery = std::function<double()>;

  PredAlertLimitProvider();
  explicit PredAlertLimitProvider(Config config);

  void setConfig(Config config);
  const Config& config() const { return config_; }
  void setEnvironment(OccupancyQuery occupancy_query,
                      MapRegionQuery map_region_query,
                      ResolutionQuery resolution_query);

  PredAlertLimitSample evaluate(const Eigen::Vector3d& p_w,
                                double query_time_s,
                                double current_hal,
                                double current_val) const;

  static const char* modeName(PredAlertLimitMode mode);
  static PredAlertLimitMode modeFromString(const std::string& value);

 private:
  static bool finite(double value);
  static double clampPositive(double value, double min_value, double max_value);
  PredAlertLimitSample evaluateCurrent(double current_hal,
                                       double current_val) const;
  PredAlertLimitSample evaluateConstant() const;
  PredAlertLimitSample evaluateVerticalBound(const Eigen::Vector3d& p_w,
                                             double hal) const;
  PredAlertLimitSample evaluateOccupancyClearance(
      const Eigen::Vector3d& p_w) const;
  bool mapRegion(Eigen::Vector3d* origin, Eigen::Vector3d* size) const;
  double verticalLimit(const Eigen::Vector3d& p_w,
                       const Eigen::Vector3d& origin,
                       const Eigen::Vector3d& size) const;
  double horizontalClearance(const Eigen::Vector3d& p_w) const;

  Config config_;
  OccupancyQuery occupancy_query_;
  MapRegionQuery map_region_query_;
  ResolutionQuery resolution_query_;
};

struct P5GateStatus {
  P5GateAction action = P5GateAction::OK;
  P5GateAction raw_action = P5GateAction::OK;
  P5GateReason reason = P5GateReason::DISABLED;
  P5GateReason raw_reason = P5GateReason::DISABLED;
  std::string current_reason;
  std::string future_reason;
  std::string current_integrity_source = "FUSED";
  std::vector<std::string> active_reasons;
  double current_im_h = std::numeric_limits<double>::quiet_NaN();
  double current_im_v = std::numeric_limits<double>::quiet_NaN();
  double current_im_min = std::numeric_limits<double>::quiet_NaN();
  double future_min_im = std::numeric_limits<double>::quiet_NaN();
  double first_bad_tau = std::numeric_limits<double>::quiet_NaN();
  double bad_ratio = 0.0;
  double unknown_ratio = 0.0;
  double current_integrity_age_s = std::numeric_limits<double>::quiet_NaN();
  uint64_t field_generation_id = 0;
  double field_age_s = std::numeric_limits<double>::quiet_NaN();
  double current_stale_duration_s = 0.0;
  double current_low_margin_duration_s = 0.0;
  double future_unknown_duration_s = 0.0;
  int final_gate_fail_count = 0;
  double final_gate_fail_duration_s = 0.0;
  std::string final_gate_last_reason;
  int final_candidate_traj_id = -1;
  double final_candidate_start_time_s =
      std::numeric_limits<double>::quiet_NaN();
  int64_t final_candidate_start_time_ns =
      std::numeric_limits<int64_t>::min();
  double final_candidate_duration_s =
      std::numeric_limits<double>::quiet_NaN();
  bool final_candidate_rejected = false;
  std::string pred_al_mode;
  double pred_hal_min = std::numeric_limits<double>::quiet_NaN();
  double pred_val_min = std::numeric_limits<double>::quiet_NaN();
  int pred_al_invalid_count = 0;
  std::string pred_al_last_reason;
  int sample_count = 0;
  int bad_count = 0;
  int unknown_count = 0;
  std::vector<SafetyVizTrajectorySample> viz_samples;
};

class P5RuntimeIntegrityGate {
 public:
  struct Config {
    bool enable_runtime_gate = false;
    bool enable_final_gate = false;
    bool debug_metrics_enable = false;
    double horizon_s = 2.0;
    double sample_dt_s = 0.2;
    double current_stale_to_replan_s = 0.5;
    double current_stale_to_emergency_s = 2.0;
    double current_low_margin_to_emergency_s = 2.0;
    double future_unknown_to_emergency_s = 2.0;
    int final_gate_max_consecutive_failures = 3;
    double final_gate_max_failure_duration_s = 1.0;
    double current_replan_margin_m = 0.3;
    double current_emergency_margin_m = -0.2;
    double future_replan_margin_m = 0.3;
    double future_emergency_margin_m = -0.5;
    double max_bad_ratio = 0.25;
    double max_unknown_ratio = 0.30;
    int bad_tick_to_replan = 2;
    int good_tick_to_clear = 2;
    PredAlertLimitProvider::Config pred_alert_limit;
    std::string integrity_topic = "/iap/integrity";
    std::string status_topic = "planning/integrity_gate_status";
  };

  static Config declareAndReadConfig(const rclcpp::Node::SharedPtr& node);
  static std::unique_ptr<P5RuntimeIntegrityGate> createIfEnabled(
      const rclcpp::Node::SharedPtr& node);

  P5RuntimeIntegrityGate(rclcpp::Node::SharedPtr node,
                         Config config,
                         bool create_ros_interfaces = true);

  bool runtimeEnabled() const { return config_.enable_runtime_gate; }
  bool finalGateEnabled() const { return config_.enable_final_gate; }

  P5GateStatus evaluateRuntime(
      LocalTrajData& local_data,
      const std::shared_ptr<const iap::RiskGridSnapshot>& snapshot,
      double now_s,
      double emergency_time_s);

  P5GateStatus evaluateFinal(
      LocalTrajData& local_data,
      const std::shared_ptr<const iap::RiskGridSnapshot>& snapshot,
      double now_s,
      double emergency_time_s);

  void publishStatus(const P5GateStatus& status, const std::string& phase);

  void setCurrentIntegrityForTest(const iap::msg::IntegrityReport& msg);
  void setPredAlertLimitEnvironment(
      PredAlertLimitProvider::OccupancyQuery occupancy_query,
      PredAlertLimitProvider::MapRegionQuery map_region_query,
      PredAlertLimitProvider::ResolutionQuery resolution_query);
  void resetFinalGateFailureState();

  static const char* actionName(P5GateAction action);
  static const char* reasonName(P5GateReason reason);

 private:
  struct CurrentIntegrity {
    bool received = false;
    bool valid = false;
    double stamp_s = std::numeric_limits<double>::quiet_NaN();
    double hpl = std::numeric_limits<double>::quiet_NaN();
    double vpl = std::numeric_limits<double>::quiet_NaN();
    double hal = std::numeric_limits<double>::quiet_NaN();
    double val = std::numeric_limits<double>::quiet_NaN();
    double im = std::numeric_limits<double>::quiet_NaN();
  };

  struct EvalContext {
    bool final_gate = false;
    double now_s = std::numeric_limits<double>::quiet_NaN();
    double emergency_time_s = 1.0;
  };

  void createRosInterfaces();
  void integrityCallback(const iap::msg::IntegrityReport::ConstSharedPtr msg);
  static double stampToSec(const builtin_interfaces::msg::Time& stamp);
  static bool finite(double value);
  CurrentIntegrity currentFromMsg(
      const iap::msg::IntegrityReport& msg) const;

  P5GateStatus evaluate(
      LocalTrajData& local_data,
      const std::shared_ptr<const iap::RiskGridSnapshot>& snapshot,
      const EvalContext& context);
  P5GateStatus evaluateCurrentGate(double now_s, bool update_state);
  P5GateStatus evaluateFutureGate(
      LocalTrajData& local_data,
      const std::shared_ptr<const iap::RiskGridSnapshot>& snapshot,
      const CurrentIntegrity& current,
      const EvalContext& context);
  P5GateStatus merge(const P5GateStatus& a, const P5GateStatus& b) const;
  P5GateStatus applyDebounce(const P5GateStatus& raw, double now_s);
  P5GateStatus applyFinalGateBudget(const P5GateStatus& raw, double now_s);
  std::string toJson(const P5GateStatus& status,
                     const std::string& phase) const;

  rclcpp::Node::SharedPtr node_;
  Config config_;
  PredAlertLimitProvider pred_alert_limit_provider_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::Subscription<iap::msg::IntegrityReport>::SharedPtr integrity_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  std::shared_ptr<SafetyRvizPublisher> safety_viz_;

  mutable std::mutex mutex_;
  CurrentIntegrity current_;
  double current_problem_started_s_ = std::numeric_limits<double>::quiet_NaN();
  double current_low_margin_started_s_ =
      std::numeric_limits<double>::quiet_NaN();
  double future_unknown_started_s_ = std::numeric_limits<double>::quiet_NaN();
  int bad_ticks_ = 0;
  int good_ticks_ = 0;
  int final_gate_fail_count_ = 0;
  double final_gate_first_failure_s_ =
      std::numeric_limits<double>::quiet_NaN();
  P5GateReason final_gate_last_reason_ = P5GateReason::OK;
};

}  // namespace ego_planner

#endif
