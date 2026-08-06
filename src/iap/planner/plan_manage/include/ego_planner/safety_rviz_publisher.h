#ifndef _SAFETY_RVIZ_PUBLISHER_H_
#define _SAFETY_RVIZ_PUBLISHER_H_

#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <iap/planner/risk_grid_map.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace ego_planner {

struct SafetyVizTrajectorySample {
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  double tau_s = 0.0;
  double query_tau_s = std::numeric_limits<double>::quiet_NaN();
  double trajectory_start_time_s = std::numeric_limits<double>::quiet_NaN();
  double trajectory_duration_s = std::numeric_limits<double>::quiet_NaN();
  double trajectory_t_cur_s = std::numeric_limits<double>::quiet_NaN();
  double trajectory_t_end_s = std::numeric_limits<double>::quiet_NaN();
  double trajectory_time_remaining_s =
      std::numeric_limits<double>::quiet_NaN();
  double sample_dt_s = std::numeric_limits<double>::quiet_NaN();
  double horizon_s = std::numeric_limits<double>::quiet_NaN();
  std::string trajectory_sample_source = "runtime_committed";
  double hpl = std::numeric_limits<double>::quiet_NaN();
  double vpl = std::numeric_limits<double>::quiet_NaN();
  double hal = std::numeric_limits<double>::quiet_NaN();
  double val = std::numeric_limits<double>::quiet_NaN();
  double im_min = std::numeric_limits<double>::quiet_NaN();
  bool fixture_match = false;
  double fixture_expected_hpl = std::numeric_limits<double>::quiet_NaN();
  double fixture_expected_vpl = std::numeric_limits<double>::quiet_NaN();
  std::string fixture_expected_reason;
  bool good = false;
  bool bad = false;
  bool unknown = false;
  bool stale = false;
  std::string reason = "not_evaluated";
};

struct SafetyVizGateStatus {
  std::string phase = "runtime";
  std::string action = "OK";
  std::string reason = "ok";
  double current_im_h = std::numeric_limits<double>::quiet_NaN();
  double current_im_v = std::numeric_limits<double>::quiet_NaN();
  double current_im_min = std::numeric_limits<double>::quiet_NaN();
  double future_min_im = std::numeric_limits<double>::quiet_NaN();
  double first_bad_tau = std::numeric_limits<double>::quiet_NaN();
  double bad_ratio = 0.0;
  double unknown_ratio = 0.0;
  int sample_count = 0;
  int bad_count = 0;
  int unknown_count = 0;
  std::vector<SafetyVizTrajectorySample> samples;
};

struct SafetyVizP1Sample {
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

struct SafetyVizP1Metrics {
  int sample_count = 0;
  int hit_count = 0;
  int miss_count = 0;
  int stale_count = 0;
  double f_integrity = 0.0;
  double weighted_f_integrity = 0.0;
  double grad_ratio = 0.0;
  uint64_t snapshot_generation_id = 0;
  bool applied_to_objective = false;
  std::string fallback_reason = "not_evaluated";
};

struct SafetyVizP2Candidate {
  int candidate_id = 0;
  bool selected = false;
  bool fallback = false;
  double score = 0.0;
  double valid_ratio = 0.0;
  std::string reason = "not_evaluated";
  std::vector<Eigen::Vector3d> control_points;
};

struct SafetyVizP3ReferenceBias {
  bool local = true;
  bool used_bias = false;
  Eigen::Vector3d start = Eigen::Vector3d::Zero();
  Eigen::Vector3d end = Eigen::Vector3d::Zero();
  Eigen::Vector3d nominal_target = Eigen::Vector3d::Zero();
  Eigen::Vector3d biased_target = Eigen::Vector3d::Zero();
  std::vector<Eigen::Vector3d> biased_waypoints;
  double improvement_ratio = 0.0;
  std::string reason = "not_evaluated";
};

struct SafetyVizP4Guide {
  std::vector<Eigen::Vector3d> original_path;
  std::vector<Eigen::Vector3d> risk_path;
  std::vector<Eigen::Vector3d> selected_path;
  Eigen::Vector3d segment_start = Eigen::Vector3d::Zero();
  Eigen::Vector3d segment_end = Eigen::Vector3d::Zero();
  double path_length_ratio = 0.0;
  bool risk_selected = false;
  std::string reason = "not_evaluated";
};

class SafetyRvizPublisher {
 public:
  struct Config {
    bool enabled = true;
    bool enable_im_bars = true;
    bool enable_validity_cloud = true;
    bool enable_p1_viz = false;
    bool enable_p2_viz = false;
    bool enable_p3_viz = false;
    bool enable_p4_viz = false;
    std::string frame_id = "map";
    std::string risk_grid_health_topic = "/iap/rviz/risk_grid_health";
    std::string predicted_pl_cloud_topic = "/iap/rviz/predicted_pl_cloud";
    std::string risk_validity_cloud_topic = "/iap/rviz/risk_validity_cloud";
    std::string trajectory_samples_topic =
        "/iap/rviz/trajectory_integrity_samples";
    std::string current_traj_topic = "/iap/rviz/current_traj_integrity_colored";
    std::string p5_gate_status_topic = "/iap/rviz/p5_gate_status";
    std::string p5_current_im_bars_topic = "/iap/rviz/p5_current_im_bars";
    std::string p1_integrity_samples_topic = "/iap/rviz/p1_integrity_samples";
    std::string p1_integrity_push_vectors_topic =
        "/iap/rviz/p1_integrity_push_vectors";
    std::string p1_integrity_metrics_topic = "/iap/rviz/p1_integrity_metrics";
    std::string p2_candidate_trajectories_topic =
        "/iap/rviz/p2_candidate_trajectories";
    std::string p3_reference_bias_topic = "/iap/rviz/p3_reference_bias";
    std::string p4_astar_guides_topic = "/iap/rviz/p4_astar_guides";
    double selected_horizon_s = 1.0;
    std::string z_slice_mode = "current_altitude";
    double z_slice_half_thickness_m = 0.75;
    double publish_rate_hz = 2.0;
    int max_cloud_points = 20000;
  };

  static Config declareAndReadConfig(const rclcpp::Node::SharedPtr& node);

  explicit SafetyRvizPublisher(rclcpp::Node::SharedPtr node, Config config);

  bool enabled() const { return config_.enabled; }

  void publishRiskGridHealth(const iap::RiskGridHealth& health,
                             double now_s);
  void publishPredictedPLCloud(
      const std::shared_ptr<const iap::RiskGridSnapshot>& snapshot,
      double current_altitude_m,
      double now_s,
      bool force = false);
  void publishRiskValidityCloud(
      const std::shared_ptr<const iap::RiskGridSnapshot>& snapshot,
      double current_altitude_m,
      double now_s);
  void publishP5GateStatus(const SafetyVizGateStatus& status,
                           double now_s);
  void publishP1IntegrityViz(const std::vector<SafetyVizP1Sample>& samples,
                             const SafetyVizP1Metrics& metrics,
                             double now_s);
  void publishP2Candidates(const std::vector<SafetyVizP2Candidate>& candidates,
                           double now_s);
  void publishP3ReferenceBias(const SafetyVizP3ReferenceBias& bias,
                              double now_s);
  void publishP4Guides(const std::vector<SafetyVizP4Guide>& guides,
                       double now_s);

  static visualization_msgs::msg::MarkerArray buildRiskGridHealthMarkers(
      const iap::RiskGridHealth& health,
      const Config& config,
      const rclcpp::Time& stamp);
  static sensor_msgs::msg::PointCloud2 buildPredictedPLCloud(
      const std::shared_ptr<const iap::RiskGridSnapshot>& snapshot,
      const Config& config,
      double current_altitude_m,
      const rclcpp::Time& stamp);
  static sensor_msgs::msg::PointCloud2 buildRiskValidityCloud(
      const std::shared_ptr<const iap::RiskGridSnapshot>& snapshot,
      const Config& config,
      double current_altitude_m,
      const rclcpp::Time& stamp);
  static visualization_msgs::msg::MarkerArray buildTrajectorySampleMarkers(
      const SafetyVizGateStatus& status,
      const Config& config,
      const rclcpp::Time& stamp);
  static visualization_msgs::msg::MarkerArray buildTrajectoryLineMarkers(
      const SafetyVizGateStatus& status,
      const Config& config,
      const rclcpp::Time& stamp);
  static visualization_msgs::msg::MarkerArray buildP5GateStatusMarkers(
      const SafetyVizGateStatus& status,
      const Config& config,
      const rclcpp::Time& stamp);
  static visualization_msgs::msg::MarkerArray buildP5CurrentImBarMarkers(
      const SafetyVizGateStatus& status,
      const Config& config,
      const rclcpp::Time& stamp);
  static visualization_msgs::msg::MarkerArray buildP1IntegritySampleMarkers(
      const std::vector<SafetyVizP1Sample>& samples,
      const Config& config,
      const rclcpp::Time& stamp);
  static visualization_msgs::msg::MarkerArray buildP1PushVectorMarkers(
      const std::vector<SafetyVizP1Sample>& samples,
      const Config& config,
      const rclcpp::Time& stamp);
  static visualization_msgs::msg::MarkerArray buildP1MetricsMarkers(
      const SafetyVizP1Metrics& metrics,
      const Config& config,
      const rclcpp::Time& stamp);
  static visualization_msgs::msg::MarkerArray buildP2CandidateMarkers(
      const std::vector<SafetyVizP2Candidate>& candidates,
      const Config& config,
      const rclcpp::Time& stamp);
  static visualization_msgs::msg::MarkerArray buildP3ReferenceBiasMarkers(
      const SafetyVizP3ReferenceBias& bias,
      const Config& config,
      const rclcpp::Time& stamp);
  static visualization_msgs::msg::MarkerArray buildP4GuideMarkers(
      const std::vector<SafetyVizP4Guide>& guides,
      const Config& config,
      const rclcpp::Time& stamp);

 private:
  bool shouldPublish(double now_s, double* last_publish_s) const;

  rclcpp::Node::SharedPtr node_;
  Config config_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      risk_grid_health_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      predicted_pl_cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      risk_validity_cloud_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      trajectory_samples_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      current_traj_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      p5_gate_status_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      p5_current_im_bars_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      p1_integrity_samples_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      p1_integrity_push_vectors_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      p1_integrity_metrics_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      p2_candidate_trajectories_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      p3_reference_bias_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      p4_astar_guides_pub_;
  double last_grid_publish_s_ = std::numeric_limits<double>::quiet_NaN();
  double last_validity_publish_s_ = std::numeric_limits<double>::quiet_NaN();
  double last_p5_publish_s_ = std::numeric_limits<double>::quiet_NaN();
  double last_p1_publish_s_ = std::numeric_limits<double>::quiet_NaN();
  double last_p2_publish_s_ = std::numeric_limits<double>::quiet_NaN();
  double last_p3_publish_s_ = std::numeric_limits<double>::quiet_NaN();
  double last_p4_publish_s_ = std::numeric_limits<double>::quiet_NaN();
};

}  // namespace ego_planner

#endif
