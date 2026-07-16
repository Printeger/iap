#ifndef _P0_RISK_GRID_RUNTIME_H_
#define _P0_RISK_GRID_RUNTIME_H_

#include <cstddef>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <ego_planner/safety_rviz_publisher.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <gnss_comm/msg/gnss_ephem_msg.hpp>
#include <gnss_comm/msg/gnss_glo_ephem_msg.hpp>
#include <gnss_comm/msg/gnss_ionosphere_parameter.hpp>
#include <gnss_comm/msg/gnss_meas_msg.hpp>
#include <gnss_comm/gnss_utility.hpp>
#include <iap/msg/integrity_report.hpp>
#include <iap/planner/integrity_snapshot.hpp>
#include <iap/planner/risk_grid_map.hpp>
#include <iap/predictor/predictor_types.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/string.hpp>

namespace ego_planner {

class P0RiskGridRuntime {
 public:
  struct P0_6FixtureConfig {
    bool enabled = false;
    std::string name;
    double x_min_m = -1.5;
    double x_max_m = 1.5;
    double y_min_m = -0.75;
    double y_max_m = 0.75;
    double z_min_m = 1.0;
    double z_max_m = 2.0;
    double raw_hpl_m = 1.0;
    double raw_vpl_m = 1.2;
    double raw_c_pi = 1.2;
    double low_raw_cost_threshold = 2.0;
  };

  struct Config {
    bool enable_risk_grid = false;
    bool debug_metrics_enable = false;
    iap::RiskGridMapParams grid;
    std::string odom_topic = "/drone_0_visual_slam/odom";
    std::string integrity_topic = "/iap/integrity";
    std::string range_meas_topic = "/ublox_driver/range_meas";
    std::string ephem_topic = "/ublox_driver/ephem";
    std::string glo_ephem_topic = "/ublox_driver/glo_ephem";
    std::string receiver_lla_topic = "/ublox_driver/receiver_lla";
    std::string iono_topic = "/ublox_driver/iono_params";
    std::string map_topic = "/map_generator/global_cloud";
    // This is an evidence topic, not a debug topic.  Keep it absolute so a
    // namespaced planner cannot silently move it away from the bag contract.
    std::string health_topic = "/planning/risk_grid_health";
    double gnss_epoch_max_age_s = 2.0;
    iap::PredictorSourceMode predictor_source_mode =
        iap::PredictorSourceMode::Fusion;
    iap::PredictorGnssEpochPolicy predictor_gnss_epoch_policy =
        iap::PredictorGnssEpochPolicy::Auto;
    bool predictor_use_current_integrity_prior = true;
    bool predictor_conservative_max_with_gnss = false;
    bool predictor_lidar_legacy_observability = true;
    double predictor_lidar_fim_radius_m =
        iap::LidarObservabilityFim::Params{}.fim_radius_m;
    int predictor_requested_worker_count = 1;
    int predictor_effective_worker_count = 1;
    P0_6FixtureConfig p0_6_fixture;
  };

  static Config declareAndReadConfig(const rclcpp::Node::SharedPtr& node);
  static std::unique_ptr<P0RiskGridRuntime> createIfEnabled(
      const rclcpp::Node::SharedPtr& node);

  P0RiskGridRuntime(
      rclcpp::Node::SharedPtr node,
      Config config,
      std::unique_ptr<iap::RiskPredictionProvider> provider = nullptr);

  bool enabled() const { return config_.enable_risk_grid; }
  const iap::RiskGridMap& riskGrid() const { return risk_grid_; }
  iap::RiskGridMap& riskGrid() { return risk_grid_; }
  std::shared_ptr<const iap::RiskGridSnapshot> acquireSnapshot() const {
    return risk_grid_.acquireSnapshot();
  }
  iap::RiskGridHealth health() const;
  bool refreshOnceForTest();
  void setOccupancyPredicate(iap::RiskGridMap::OccupancyPredicate predicate);

 private:
  friend class P0RiskGridRuntimeStampTest;

  void createRosInterfaces();
  void refreshTimerCallback();
  void healthTimerCallback();
  void publishHealth(const iap::RiskGridHealth& health, double now_s);

  struct HealthPublicationState {
    double refresh_stamp_s = std::numeric_limits<double>::quiet_NaN();
    double refresh_start_stamp_s = std::numeric_limits<double>::quiet_NaN();
    double refresh_end_stamp_s = std::numeric_limits<double>::quiet_NaN();
    double health_callback_stamp_s = std::numeric_limits<double>::quiet_NaN();
    double publish_stamp_s = std::numeric_limits<double>::quiet_NaN();
    double refresh_start_steady_s = std::numeric_limits<double>::quiet_NaN();
    double refresh_end_steady_s = std::numeric_limits<double>::quiet_NaN();
    double health_callback_steady_s = std::numeric_limits<double>::quiet_NaN();
    double publish_steady_s = std::numeric_limits<double>::quiet_NaN();
    double last_grid_stamp_s = std::numeric_limits<double>::quiet_NaN();
    double refresh_elapsed_ms = std::numeric_limits<double>::quiet_NaN();
    double health_callback_duration_ms = std::numeric_limits<double>::quiet_NaN();
    double health_callback_queue_delay_ms = std::numeric_limits<double>::quiet_NaN();
    double health_state_mutex_wait_ms = std::numeric_limits<double>::quiet_NaN();
    double health_state_mutex_hold_ms = std::numeric_limits<double>::quiet_NaN();
    std::size_t refresh_query_count = 0;
    bool snapshot_available = false;
  };
  HealthPublicationState healthPublicationStateSnapshot() const;

  void odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr msg);
  void integrityCallback(const iap::msg::IntegrityReport::ConstSharedPtr msg);
  void rangeCallback(const gnss_comm::msg::GnssMeasMsg::ConstSharedPtr msg);
  void ephemCallback(const gnss_comm::msg::GnssEphemMsg::ConstSharedPtr msg);
  void gloEphemCallback(
      const gnss_comm::msg::GnssGloEphemMsg::ConstSharedPtr msg);
  void receiverLlaCallback(
      const sensor_msgs::msg::NavSatFix::ConstSharedPtr msg);
  void ionoCallback(
      const gnss_comm::msg::GnssIonosphereParameter::ConstSharedPtr msg);
  void cloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);

  iap::CurrentIntegrityState currentFromMsg(
      const iap::msg::IntegrityReport& msg) const;
  const iap::GnssEpoch* activeGnssEpoch(double query_stamp) const;
  Eigen::Matrix3d currentPriorInformation(
      const iap::CurrentIntegrityState& current) const;
  bool buildSnapshot(double now_s, iap::IntegritySnapshot* snapshot) const;
  iap::RiskGridHealth addLidarPredictorInputHealth(
      iap::RiskGridHealth health) const;
  bool p0_6_fixture_occupied(const Eigen::Vector3d& pos) const;
  iap::RiskGridMap::OccupancyPredicate combinedOccupancyPredicate() const;
  double currentMessageStamp() const;
  double currentRefreshStamp() const;

  rclcpp::Node::SharedPtr node_;
  Config config_;
  iap::RiskGridMap risk_grid_;
  std::unique_ptr<iap::RiskPredictionProvider> provider_;
  iap::RiskGridMap::OccupancyPredicate occupancy_predicate_;
  iap::IntegritySnapshotBuilder snapshot_builder_;

  // Inputs, heavy refresh, and health publication deliberately use distinct
  // execution paths.  Refresh may take longer than a sensor period, but must
  // never prevent the next input state from being recorded.
  rclcpp::CallbackGroup::SharedPtr input_callback_group_;
  rclcpp::CallbackGroup::SharedPtr refresh_callback_group_;
  rclcpp::CallbackGroup::SharedPtr health_callback_group_;
  rclcpp::TimerBase::SharedPtr refresh_timer_;
  rclcpp::TimerBase::SharedPtr health_timer_;
  std::shared_ptr<SafetyRvizPublisher> safety_viz_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr health_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<iap::msg::IntegrityReport>::SharedPtr integrity_sub_;
  rclcpp::Subscription<gnss_comm::msg::GnssMeasMsg>::SharedPtr range_sub_;
  rclcpp::Subscription<gnss_comm::msg::GnssEphemMsg>::SharedPtr ephem_sub_;
  rclcpp::Subscription<gnss_comm::msg::GnssGloEphemMsg>::SharedPtr
      glo_ephem_sub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr
      receiver_lla_sub_;
  rclcpp::Subscription<gnss_comm::msg::GnssIonosphereParameter>::SharedPtr
      iono_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;

  double latest_odom_stamp_ = std::numeric_limits<double>::quiet_NaN();
  Eigen::Vector3d latest_odom_p_ =
      Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
  Eigen::Quaterniond latest_odom_q_ = Eigen::Quaterniond::Identity();
  bool latest_odom_pose_valid_ = false;
  double latest_map_stamp_ = std::numeric_limits<double>::quiet_NaN();
  iap::CurrentIntegrityState latest_current_;
  bool latest_current_valid_ = false;
  double last_refresh_stamp_s_ = std::numeric_limits<double>::quiet_NaN();
  double last_grid_stamp_s_ = std::numeric_limits<double>::quiet_NaN();
  double last_refresh_elapsed_ms_ = std::numeric_limits<double>::quiet_NaN();
  bool last_snapshot_available_ = false;
  std::size_t last_refresh_query_count_ = 0;
  double last_refresh_start_stamp_s_ = std::numeric_limits<double>::quiet_NaN();
  double last_refresh_end_stamp_s_ = std::numeric_limits<double>::quiet_NaN();
  double last_health_callback_stamp_s_ = std::numeric_limits<double>::quiet_NaN();
  double last_publish_stamp_s_ = std::numeric_limits<double>::quiet_NaN();
  double last_refresh_start_steady_s_ = std::numeric_limits<double>::quiet_NaN();
  double last_refresh_end_steady_s_ = std::numeric_limits<double>::quiet_NaN();
  double last_health_callback_steady_s_ = std::numeric_limits<double>::quiet_NaN();
  double last_publish_steady_s_ = std::numeric_limits<double>::quiet_NaN();
  double last_health_callback_duration_ms_ = std::numeric_limits<double>::quiet_NaN();
  double last_health_callback_queue_delay_ms_ = std::numeric_limits<double>::quiet_NaN();
  double last_health_state_mutex_wait_ms_ = std::numeric_limits<double>::quiet_NaN();
  double last_health_state_mutex_hold_ms_ = std::numeric_limits<double>::quiet_NaN();
  bool origin_set_ = false;
  Eigen::Vector3d origin_ecef_ = Eigen::Vector3d::Zero();
  std::unordered_map<uint32_t, gnss_comm::EphemPtr> ephem_cache_;
  std::unordered_map<uint32_t, gnss_comm::GloEphemPtr> glo_ephem_cache_;
  std::vector<double> iono_params_;
  std::optional<iap::GnssEpoch> latest_epoch_;

  mutable std::mutex lidar_predictor_input_mutex_;
  std::shared_ptr<const std::vector<Eigen::Vector3d>>
      latest_lidar_map_points_;
  std::shared_ptr<const std::vector<iap::LidarFimPrimitive>>
      latest_lidar_fim_primitives_;
  iap::LidarFimPrimitiveGenerationDiagnostics
      latest_lidar_fim_diagnostics_;
  std::size_t latest_lidar_map_point_count_ = 0;
  std::size_t latest_lidar_fim_primitive_count_ = 0;
  std::size_t latest_lidar_fim_valid_normal_count_ = 0;
  std::string latest_lidar_fim_fallback_reason_ = "not_evaluated";
  mutable std::mutex health_state_mutex_;
};

}  // namespace ego_planner

#endif
