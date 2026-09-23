#include <ego_planner/safety_rviz_publisher.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <iomanip>
#include <limits>
#include <sstream>

#include <geometry_msgs/msg/point.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/color_rgba.hpp>

namespace ego_planner {
namespace {

template <typename T>
T declare_or_get(const rclcpp::Node::SharedPtr& node,
                 const std::string& name,
                 const T& default_value) {
  if (!node) {
    return default_value;
  }
  if (!node->has_parameter(name)) {
    return node->declare_parameter<T>(name, default_value);
  }
  return node->get_parameter(name).get_value<T>();
}

bool finite(double value) {
  return std::isfinite(value);
}

bool valid_stamp_s(double value) {
  return finite(value) && value >= 0.0;
}

rclcpp::Time stamp_from_seconds(const rclcpp::Node::SharedPtr& node,
                                const double stamp_s) {
  (void)node;
  if (valid_stamp_s(stamp_s)) {
    auto sec = static_cast<int32_t>(std::floor(stamp_s));
    auto nsec = static_cast<uint32_t>(
        std::llround((stamp_s - static_cast<double>(sec)) * 1.0e9));
    if (nsec >= 1000000000u) {
      ++sec;
      nsec = 0u;
    }
    return rclcpp::Time(sec, nsec, RCL_ROS_TIME);
  }
  return rclcpp::Time(0, 0, RCL_ROS_TIME);
}

std::string fmt_num(double value, int precision = 2) {
  if (!finite(value)) {
    return "n/a";
  }
  std::ostringstream oss;
  oss << std::fixed << std::setprecision(precision) << value;
  return oss.str();
}

std_msgs::msg::ColorRGBA color(float r, float g, float b, float a = 1.0f) {
  std_msgs::msg::ColorRGBA c;
  c.r = r;
  c.g = g;
  c.b = b;
  c.a = a;
  return c;
}

std_msgs::msg::ColorRGBA health_color(const iap::RiskGridHealth& health) {
  if (health.ready && !health.stale) {
    return color(0.1f, 0.85f, 0.25f, 1.0f);
  }
  if (health.stale && health.generation_id > 0) {
    return color(1.0f, 0.75f, 0.1f, 1.0f);
  }
  return color(1.0f, 0.15f, 0.1f, 1.0f);
}

std_msgs::msg::ColorRGBA action_color(const std::string& action) {
  if (action == "OK") {
    return color(0.1f, 0.85f, 0.25f, 1.0f);
  }
  if (action == "REQUEST_REPLAN") {
    return color(1.0f, 0.55f, 0.05f, 1.0f);
  }
  if (action == "REQUEST_EMERGENCY_STOP_CANDIDATE") {
    return color(1.0f, 0.05f, 0.04f, 1.0f);
  }
  return color(0.6f, 0.6f, 0.6f, 1.0f);
}

std_msgs::msg::ColorRGBA sample_color(const SafetyVizTrajectorySample& sample) {
  if (sample.stale) {
    return color(1.0f, 0.75f, 0.05f, 0.95f);
  }
  if (sample.unknown) {
    return color(0.55f, 0.55f, 0.55f, 0.9f);
  }
  if (sample.bad) {
    return color(1.0f, 0.05f, 0.04f, 1.0f);
  }
  return color(0.1f, 0.85f, 0.25f, 0.95f);
}

std_msgs::msg::ColorRGBA risk_ratio_color(double risk_ratio,
                                          bool valid,
                                          bool unknown,
                                          bool stale,
                                          bool occupied) {
  if (occupied) {
    return color(0.65f, 0.05f, 0.85f, 1.0f);
  }
  if (stale) {
    return color(0.0f, 0.0f, 0.0f, 0.9f);
  }
  if (unknown || !valid || !finite(risk_ratio)) {
    return color(0.55f, 0.55f, 0.55f, 0.65f);
  }
  if (risk_ratio < 0.5) {
    const float t = static_cast<float>(std::clamp(
        risk_ratio / 0.5, 0.0, 1.0));
    return color(0.05f + 0.05f * t, 0.35f + 0.50f * t,
                 1.0f - 0.75f * t, 0.9f);
  }
  if (risk_ratio < 0.8) {
    return color(1.0f, 0.8f, 0.05f, 0.95f);
  }
  if (risk_ratio < 1.0) {
    return color(1.0f, 0.42f, 0.02f, 0.95f);
  }
  return color(1.0f, 0.02f, 0.02f, 1.0f);
}

std_msgs::msg::ColorRGBA validity_color(const iap::RiskVoxel& voxel) {
  if ((voxel.source_flags & iap::RISK_GRID_SOURCE_OCCUPIED_SKIP) != 0u ||
      (voxel.occupancy && voxel.occupancy->inflated_occupied)) {
    return color(0.65f, 0.05f, 0.85f, 1.0f);
  }
  if (voxel.stale) {
    return color(0.12f, 0.12f, 0.12f, 0.8f);
  }
  if (voxel.unknown || !voxel.valid) {
    return color(0.55f, 0.55f, 0.55f, 0.75f);
  }
  if (voxel.source_flags != 0u) {
    return color(0.1f, 0.85f, 0.25f, 0.9f);
  }
  return color(0.0f, 0.85f, 0.95f, 0.85f);
}

std_msgs::msg::ColorRGBA margin_color(double margin) {
  if (!finite(margin)) {
    return color(0.45f, 0.45f, 0.45f, 0.85f);
  }
  if (margin < 0.0) {
    return color(1.0f, 0.05f, 0.04f, 0.95f);
  }
  if (margin < 0.3) {
    return color(1.0f, 0.78f, 0.05f, 0.95f);
  }
  return color(0.1f, 0.85f, 0.25f, 0.9f);
}

float packed_rgb_float(const std_msgs::msg::ColorRGBA& c) {
  const auto r = static_cast<uint32_t>(
      std::clamp(c.r, 0.0f, 1.0f) * 255.0f);
  const auto g = static_cast<uint32_t>(
      std::clamp(c.g, 0.0f, 1.0f) * 255.0f);
  const auto b = static_cast<uint32_t>(
      std::clamp(c.b, 0.0f, 1.0f) * 255.0f);
  const uint32_t packed = (r << 16) | (g << 8) | b;
  float out = 0.0f;
  std::memcpy(&out, &packed, sizeof(float));
  return out;
}

double percentile(std::vector<double> values, const double probability) {
  if (values.empty()) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  std::sort(values.begin(), values.end());
  const double position = std::clamp(probability, 0.0, 1.0) *
      static_cast<double>(values.size() - 1u);
  const auto lower = static_cast<std::size_t>(std::floor(position));
  const auto upper = static_cast<std::size_t>(std::ceil(position));
  const double alpha = position - static_cast<double>(lower);
  return (1.0 - alpha) * values[lower] + alpha * values[upper];
}

std_msgs::msg::ColorRGBA relative_risk_color(
    const double value, const double p5, const double p95) {
  if (!finite(value) || !finite(p5) || !finite(p95)) {
    return color(0.55f, 0.55f, 0.55f, 0.65f);
  }
  const double denominator = std::max(1.0e-12, p95 - p5);
  const float t = static_cast<float>(std::clamp(
      (value - p5) / denominator, 0.0, 1.0));
  return color(t, 0.15f + 0.7f * (1.0f - std::abs(2.0f * t - 1.0f)),
               1.0f - t, 0.9f);
}

uint8_t floor_source_code(const std::string& source) {
  if (source == "gnss_advisory") {
    return 1u;
  }
  if (source == "current_prior") {
    return 2u;
  }
  return 0u;
}

int p4_channel_marker_base(const uint64_t channel_id) {
  // Two adjacent IDs belong to one stable channel (curve + label). Keep the
  // prefix marker range separate and fold only at a large prime boundary.
  return 100 + static_cast<int>((channel_id % 1000000007ULL) * 2ULL);
}

visualization_msgs::msg::Marker base_marker(
    const SafetyRvizPublisher::Config& config,
    const rclcpp::Time& stamp,
    const std::string& ns,
    int id,
    int type) {
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = config.frame_id;
  marker.header.stamp = stamp;
  marker.ns = ns;
  marker.id = id;
  marker.type = type;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.lifetime = rclcpp::Duration::from_seconds(1.5);
  return marker;
}

geometry_msgs::msg::Point point_msg(const Eigen::Vector3d& p) {
  geometry_msgs::msg::Point out;
  out.x = p.x();
  out.y = p.y();
  out.z = p.z();
  return out;
}

int selected_horizon_index(const iap::RiskGridSnapshot& snapshot,
                           double selected_horizon_s) {
  const auto& horizons = snapshot.params().horizons_s;
  if (horizons.empty()) {
    return -1;
  }
  int best = 0;
  double best_error = std::abs(horizons.front() - selected_horizon_s);
  for (int i = 1; i < static_cast<int>(horizons.size()); ++i) {
    const double error = std::abs(horizons[static_cast<std::size_t>(i)] -
                                  selected_horizon_s);
    if (error < best_error) {
      best = i;
      best_error = error;
    }
  }
  return best;
}

}  // namespace

SafetyRvizPublisher::Config SafetyRvizPublisher::declareAndReadConfig(
    const rclcpp::Node::SharedPtr& node) {
  Config config;
  config.enabled =
      declare_or_get<bool>(node, "planner_enable_safety_viz", config.enabled);
  config.enable_im_bars = declare_or_get<bool>(
      node, "safety_viz.enable_im_bars", config.enable_im_bars);
  config.enable_validity_cloud = declare_or_get<bool>(
      node, "safety_viz.enable_validity_cloud", config.enable_validity_cloud);
  config.enable_source_diagnostic_clouds = declare_or_get<bool>(
      node, "safety_viz.enable_source_diagnostic_clouds",
      config.enable_source_diagnostic_clouds);
  config.enable_p1_viz = declare_or_get<bool>(
      node, "safety_viz.enable_p1_viz", config.enable_p1_viz);
  config.enable_p2_viz = declare_or_get<bool>(
      node, "safety_viz.enable_p2_viz", config.enable_p2_viz);
  config.enable_p3_viz = declare_or_get<bool>(
      node, "safety_viz.enable_p3_viz", config.enable_p3_viz);
  config.enable_p4_viz = declare_or_get<bool>(
      node, "safety_viz.enable_p4_viz", config.enable_p4_viz);
  config.frame_id =
      declare_or_get<std::string>(node, "safety_viz.frame_id", config.frame_id);
  config.risk_grid_health_topic = declare_or_get<std::string>(
      node, "safety_viz.risk_grid_health_topic",
      config.risk_grid_health_topic);
  config.predicted_pl_cloud_topic = declare_or_get<std::string>(
      node, "safety_viz.predicted_pl_cloud_topic",
      config.predicted_pl_cloud_topic);
  config.risk_validity_cloud_topic = declare_or_get<std::string>(
      node, "safety_viz.risk_validity_cloud_topic",
      config.risk_validity_cloud_topic);
  config.relative_risk_cloud_topic = declare_or_get<std::string>(
      node, "safety_viz.relative_risk_cloud_topic",
      config.relative_risk_cloud_topic);
  config.gnss_risk_cloud_topic = declare_or_get<std::string>(
      node, "safety_viz.gnss_risk_cloud_topic", config.gnss_risk_cloud_topic);
  config.lidar_risk_cloud_topic = declare_or_get<std::string>(
      node, "safety_viz.lidar_risk_cloud_topic",
      config.lidar_risk_cloud_topic);
  config.fim_risk_cloud_topic = declare_or_get<std::string>(
      node, "safety_viz.fim_risk_cloud_topic", config.fim_risk_cloud_topic);
  config.planning_geometry_topic = declare_or_get<std::string>(
      node, "safety_viz.planning_geometry_topic",
      config.planning_geometry_topic);
  config.trajectory_samples_topic = declare_or_get<std::string>(
      node, "safety_viz.trajectory_samples_topic",
      config.trajectory_samples_topic);
  config.current_traj_topic = declare_or_get<std::string>(
      node, "safety_viz.current_traj_topic", config.current_traj_topic);
  config.p5_gate_status_topic = declare_or_get<std::string>(
      node, "safety_viz.p5_gate_status_topic",
      config.p5_gate_status_topic);
  config.p5_current_im_bars_topic = declare_or_get<std::string>(
      node, "safety_viz.p5_current_im_bars_topic",
      config.p5_current_im_bars_topic);
  config.p1_integrity_samples_topic = declare_or_get<std::string>(
      node, "safety_viz.p1_integrity_samples_topic",
      config.p1_integrity_samples_topic);
  config.p1_integrity_push_vectors_topic = declare_or_get<std::string>(
      node, "safety_viz.p1_integrity_push_vectors_topic",
      config.p1_integrity_push_vectors_topic);
  config.p1_integrity_metrics_topic = declare_or_get<std::string>(
      node, "safety_viz.p1_integrity_metrics_topic",
      config.p1_integrity_metrics_topic);
  config.p2_candidate_trajectories_topic = declare_or_get<std::string>(
      node, "safety_viz.p2_candidate_trajectories_topic",
      config.p2_candidate_trajectories_topic);
  config.p3_reference_bias_topic = declare_or_get<std::string>(
      node, "safety_viz.p3_reference_bias_topic",
      config.p3_reference_bias_topic);
  config.p4_astar_guides_topic = declare_or_get<std::string>(
      node, "safety_viz.p4_astar_guides_topic",
      config.p4_astar_guides_topic);
  config.p4_topology_channels_topic = declare_or_get<std::string>(
      node, "safety_viz.p4_topology_channels_topic",
      config.p4_topology_channels_topic);
  config.selected_horizon_s = declare_or_get<double>(
      node, "safety_viz.selected_horizon_s", config.selected_horizon_s);
  config.z_slice_mode = declare_or_get<std::string>(
      node, "safety_viz.z_slice_mode", config.z_slice_mode);
  config.z_slice_half_thickness_m = declare_or_get<double>(
      node, "safety_viz.z_slice_half_thickness_m",
      config.z_slice_half_thickness_m);
  config.publish_rate_hz = declare_or_get<double>(
      node, "safety_viz.publish_rate_hz", config.publish_rate_hz);
  config.max_cloud_points = declare_or_get<int>(
      node, "safety_viz.max_cloud_points", config.max_cloud_points);
  config.z_slice_half_thickness_m =
      std::max(0.0, config.z_slice_half_thickness_m);
  config.publish_rate_hz = std::max(0.1, config.publish_rate_hz);
  config.max_cloud_points = std::max(1, config.max_cloud_points);
  return config;
}

SafetyRvizPublisher::SafetyRvizPublisher(rclcpp::Node::SharedPtr node,
                                         Config config)
    : node_(std::move(node)), config_(std::move(config)) {
  if (!node_ || !config_.enabled) {
    return;
  }
  risk_grid_health_pub_ =
      node_->create_publisher<visualization_msgs::msg::MarkerArray>(
          config_.risk_grid_health_topic, 10);
  predicted_pl_cloud_pub_ =
      node_->create_publisher<sensor_msgs::msg::PointCloud2>(
          config_.predicted_pl_cloud_topic, rclcpp::QoS(1).best_effort());
  if (config_.enable_validity_cloud) {
    risk_validity_cloud_pub_ =
        node_->create_publisher<sensor_msgs::msg::PointCloud2>(
            config_.risk_validity_cloud_topic, rclcpp::QoS(1).best_effort());
  }
  if (config_.enable_source_diagnostic_clouds) {
    relative_risk_cloud_pub_ =
        node_->create_publisher<sensor_msgs::msg::PointCloud2>(
            config_.relative_risk_cloud_topic,
            rclcpp::QoS(1).best_effort());
    gnss_risk_cloud_pub_ =
        node_->create_publisher<sensor_msgs::msg::PointCloud2>(
            config_.gnss_risk_cloud_topic, rclcpp::QoS(1).best_effort());
    lidar_risk_cloud_pub_ =
        node_->create_publisher<sensor_msgs::msg::PointCloud2>(
            config_.lidar_risk_cloud_topic, rclcpp::QoS(1).best_effort());
    fim_risk_cloud_pub_ =
        node_->create_publisher<sensor_msgs::msg::PointCloud2>(
            config_.fim_risk_cloud_topic, rclcpp::QoS(1).best_effort());
    planning_geometry_pub_ =
        node_->create_publisher<visualization_msgs::msg::MarkerArray>(
            config_.planning_geometry_topic, 1);
  }
  trajectory_samples_pub_ =
      node_->create_publisher<visualization_msgs::msg::MarkerArray>(
          config_.trajectory_samples_topic, 10);
  current_traj_pub_ =
      node_->create_publisher<visualization_msgs::msg::MarkerArray>(
          config_.current_traj_topic, 10);
  p5_gate_status_pub_ =
      node_->create_publisher<visualization_msgs::msg::MarkerArray>(
          config_.p5_gate_status_topic, 10);
  if (config_.enable_im_bars) {
    p5_current_im_bars_pub_ =
        node_->create_publisher<visualization_msgs::msg::MarkerArray>(
            config_.p5_current_im_bars_topic, 10);
  }
  if (config_.enable_p1_viz) {
    p1_integrity_samples_pub_ =
        node_->create_publisher<visualization_msgs::msg::MarkerArray>(
            config_.p1_integrity_samples_topic, 10);
    p1_integrity_push_vectors_pub_ =
        node_->create_publisher<visualization_msgs::msg::MarkerArray>(
            config_.p1_integrity_push_vectors_topic, 10);
    p1_integrity_metrics_pub_ =
        node_->create_publisher<visualization_msgs::msg::MarkerArray>(
            config_.p1_integrity_metrics_topic, 10);
  }
  if (config_.enable_p2_viz) {
    p2_candidate_trajectories_pub_ =
        node_->create_publisher<visualization_msgs::msg::MarkerArray>(
            config_.p2_candidate_trajectories_topic, 10);
  }
  if (config_.enable_p3_viz) {
    p3_reference_bias_pub_ =
        node_->create_publisher<visualization_msgs::msg::MarkerArray>(
            config_.p3_reference_bias_topic, 10);
  }
  if (config_.enable_p4_viz) {
    p4_astar_guides_pub_ =
        node_->create_publisher<visualization_msgs::msg::MarkerArray>(
            config_.p4_astar_guides_topic, 10);
    p4_topology_channels_pub_ =
        node_->create_publisher<visualization_msgs::msg::MarkerArray>(
            config_.p4_topology_channels_topic, 10);
  }
}

bool SafetyRvizPublisher::shouldPublish(double now_s,
                                        double* last_publish_s) const {
  if (!config_.enabled || last_publish_s == nullptr || !valid_stamp_s(now_s)) {
    return false;
  }
  if (!finite(*last_publish_s) ||
      now_s - *last_publish_s >= 1.0 / config_.publish_rate_hz) {
    *last_publish_s = now_s;
    return true;
  }
  return false;
}

void SafetyRvizPublisher::publishRiskGridHealth(
    const iap::RiskGridHealth& health,
    double now_s) {
  if (!risk_grid_health_pub_ || !valid_stamp_s(now_s)) {
    return;
  }
  const rclcpp::Time stamp = stamp_from_seconds(node_, now_s);
  risk_grid_health_pub_->publish(
      buildRiskGridHealthMarkers(health, config_, stamp));
}

void SafetyRvizPublisher::publishPredictedPLCloud(
    const std::shared_ptr<const iap::RiskGridSnapshot>& snapshot,
    double current_altitude_m,
    double now_s,
    bool force) {
  if (!predicted_pl_cloud_pub_) {
    return;
  }
  if (force) {
    last_grid_publish_s_ = now_s;
  } else if (!shouldPublish(now_s, &last_grid_publish_s_)) {
    return;
  }
  const double snapshot_stamp_s = snapshot && valid_stamp_s(snapshot->stamp_s())
      ? snapshot->stamp_s() : now_s;
  const rclcpp::Time stamp = stamp_from_seconds(node_, snapshot_stamp_s);
  predicted_pl_cloud_pub_->publish(
      buildPredictedPLCloud(snapshot, config_, current_altitude_m, stamp));
  if (relative_risk_cloud_pub_) {
    relative_risk_cloud_pub_->publish(buildDiagnosticRiskCloud(
        snapshot, config_, current_altitude_m, stamp, "fim", true));
  }
  if (gnss_risk_cloud_pub_) {
    gnss_risk_cloud_pub_->publish(buildDiagnosticRiskCloud(
        snapshot, config_, current_altitude_m, stamp, "gnss"));
  }
  if (lidar_risk_cloud_pub_) {
    lidar_risk_cloud_pub_->publish(buildDiagnosticRiskCloud(
        snapshot, config_, current_altitude_m, stamp, "lidar"));
  }
  if (fim_risk_cloud_pub_) {
    fim_risk_cloud_pub_->publish(buildDiagnosticRiskCloud(
        snapshot, config_, current_altitude_m, stamp, "fim"));
  }
  if (planning_geometry_pub_) {
    planning_geometry_pub_->publish(buildPlanningGeometryMarkers(
        snapshot, config_, current_altitude_m, stamp));
  }
}

void SafetyRvizPublisher::publishRiskValidityCloud(
    const std::shared_ptr<const iap::RiskGridSnapshot>& snapshot,
    double current_altitude_m,
    double now_s) {
  if (!risk_validity_cloud_pub_ ||
      !shouldPublish(now_s, &last_validity_publish_s_)) {
    return;
  }
  const double snapshot_stamp_s = snapshot && valid_stamp_s(snapshot->stamp_s())
      ? snapshot->stamp_s() : now_s;
  const rclcpp::Time stamp = stamp_from_seconds(node_, snapshot_stamp_s);
  risk_validity_cloud_pub_->publish(
      buildRiskValidityCloud(snapshot, config_, current_altitude_m, stamp));
}

void SafetyRvizPublisher::publishP5GateStatus(
    const SafetyVizGateStatus& status,
    double now_s) {
  if (!shouldPublish(now_s, &last_p5_publish_s_)) {
    return;
  }
  const rclcpp::Time stamp = stamp_from_seconds(node_, now_s);
  if (trajectory_samples_pub_) {
    trajectory_samples_pub_->publish(
        buildTrajectorySampleMarkers(status, config_, stamp));
  }
  if (current_traj_pub_) {
    current_traj_pub_->publish(buildTrajectoryLineMarkers(status, config_, stamp));
  }
  if (p5_gate_status_pub_) {
    p5_gate_status_pub_->publish(
        buildP5GateStatusMarkers(status, config_, stamp));
  }
  if (p5_current_im_bars_pub_) {
    p5_current_im_bars_pub_->publish(
        buildP5CurrentImBarMarkers(status, config_, stamp));
  }
}

void SafetyRvizPublisher::publishP1IntegrityViz(
    const std::vector<SafetyVizP1Sample>& samples,
    const SafetyVizP1Metrics& metrics,
    double now_s) {
  if (!config_.enable_p1_viz ||
      !shouldPublish(now_s, &last_p1_publish_s_)) {
    return;
  }
  const rclcpp::Time stamp = stamp_from_seconds(node_, now_s);
  if (p1_integrity_samples_pub_) {
    p1_integrity_samples_pub_->publish(
        buildP1IntegritySampleMarkers(samples, config_, stamp));
  }
  if (p1_integrity_push_vectors_pub_) {
    p1_integrity_push_vectors_pub_->publish(
        buildP1PushVectorMarkers(samples, config_, stamp));
  }
  if (p1_integrity_metrics_pub_) {
    p1_integrity_metrics_pub_->publish(
        buildP1MetricsMarkers(metrics, config_, stamp));
  }
}

void SafetyRvizPublisher::publishP2Candidates(
    const std::vector<SafetyVizP2Candidate>& candidates,
    double now_s) {
  if (!p2_candidate_trajectories_pub_ ||
      !shouldPublish(now_s, &last_p2_publish_s_)) {
    return;
  }
  const rclcpp::Time stamp = stamp_from_seconds(node_, now_s);
  p2_candidate_trajectories_pub_->publish(
      buildP2CandidateMarkers(candidates, config_, stamp));
}

void SafetyRvizPublisher::publishP3ReferenceBias(
    const SafetyVizP3ReferenceBias& bias,
    double now_s) {
  if (!p3_reference_bias_pub_ || !shouldPublish(now_s, &last_p3_publish_s_)) {
    return;
  }
  const rclcpp::Time stamp = stamp_from_seconds(node_, now_s);
  p3_reference_bias_pub_->publish(
      buildP3ReferenceBiasMarkers(bias, config_, stamp));
}

void SafetyRvizPublisher::publishP4Guides(
    const std::vector<SafetyVizP4Guide>& guides,
    double now_s) {
  if ((!p4_astar_guides_pub_ && !p4_topology_channels_pub_) ||
      !valid_stamp_s(now_s)) {
    return;
  }
  const rclcpp::Time stamp = stamp_from_seconds(node_, now_s);
  if (p4_astar_guides_pub_ &&
      shouldPublish(now_s, &last_p4_publish_s_)) {
    p4_astar_guides_pub_->publish(buildP4GuideMarkers(guides, config_, stamp));
  }
  std::set<uint64_t> current_channel_ids;
  for (const auto& guide : guides)
    for (std::size_t index = 0;
         index < guide.topology_candidates.size(); ++index)
      current_channel_ids.insert(
          index < guide.topology_channel_ids.size() &&
                  guide.topology_channel_ids[index] > 0
              ? guide.topology_channel_ids[index]
              : static_cast<uint64_t>(index + 1));
  const bool has_topology_payload = std::any_of(
      guides.begin(), guides.end(), [](const SafetyVizP4Guide& guide) {
        return guide.forward_decision &&
               (!guide.topology_candidates.empty() ||
                guide.observe_more_path.size() >= 2);
      }) || !last_p4_topology_channel_ids_.empty();
  if (p4_topology_channels_pub_ && has_topology_payload &&
      shouldPublish(now_s, &last_p4_topology_publish_s_)) {
    auto markers = buildP4TopologyChannelMarkers(guides, config_, stamp);
    for (const uint64_t channel_id : last_p4_topology_channel_ids_)
      if (current_channel_ids.count(channel_id) == 0u)
        for (int offset = 0; offset < 2; ++offset) {
          auto removed = base_marker(
              config_, stamp, "p4_topology_channels",
              p4_channel_marker_base(channel_id) + offset,
              visualization_msgs::msg::Marker::LINE_STRIP);
          removed.action = visualization_msgs::msg::Marker::DELETE;
          markers.markers.push_back(std::move(removed));
        }
    p4_topology_channels_pub_->publish(markers);
    last_p4_topology_channel_ids_ = std::move(current_channel_ids);
  }
}

visualization_msgs::msg::MarkerArray
SafetyRvizPublisher::buildRiskGridHealthMarkers(
    const iap::RiskGridHealth& health,
    const Config& config,
    const rclcpp::Time& stamp) {
  visualization_msgs::msg::MarkerArray arr;
  auto marker = base_marker(config, stamp, "risk_grid_health", 0,
                            visualization_msgs::msg::Marker::TEXT_VIEW_FACING);
  marker.pose.position.x = 15.0;
  marker.pose.position.y = -8.0;
  marker.pose.position.z = 4.5;
  marker.scale.z = 0.45;
  marker.color = health_color(health);
  std::ostringstream text;
  text << "RiskGridMap\n"
       << "gen: " << health.generation_id << "\n"
       << "age: " << fmt_num(health.age_s, 2) << " s\n"
       << "valid: " << fmt_num(100.0 * health.valid_ratio, 1) << "%\n"
       << "unknown: " << fmt_num(100.0 * health.unknown_ratio, 1) << "%\n"
       << "status: "
       << (health.ready && !health.stale ? "READY"
           : health.stale && health.generation_id > 0 ? "STALE"
                                                       : "NOT_READY")
       << "\nreason: " << health.reason;
  marker.text = text.str();
  arr.markers.push_back(marker);
  return arr;
}

sensor_msgs::msg::PointCloud2 SafetyRvizPublisher::buildPredictedPLCloud(
    const std::shared_ptr<const iap::RiskGridSnapshot>& snapshot,
    const Config& config,
    double current_altitude_m,
    const rclcpp::Time& stamp) {
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.frame_id = config.frame_id;
  cloud.header.stamp = stamp;
  if (!snapshot) {
    return cloud;
  }
  const int horizon_id =
      selected_horizon_index(*snapshot, config.selected_horizon_s);
  if (horizon_id < 0) {
    return cloud;
  }

  struct Row {
    Eigen::Vector3d p = Eigen::Vector3d::Zero();
    iap::RiskVoxel voxel;
  };
  std::vector<Row> rows;
  rows.reserve(static_cast<std::size_t>(
      std::min(config.max_cloud_points, snapshot->layerVoxelCount())));
  const bool slice_all = config.z_slice_mode == "all";
  const Eigen::Vector3i dims = snapshot->voxelNum();
  for (int x = 0; x < dims.x(); ++x) {
    for (int y = 0; y < dims.y(); ++y) {
      for (int z = 0; z < dims.z(); ++z) {
        if (static_cast<int>(rows.size()) >= config.max_cloud_points) {
          break;
        }
        const Eigen::Vector3i id(x, y, z);
        const Eigen::Vector3d p = snapshot->indexToPos(id);
        if (!slice_all && finite(current_altitude_m) &&
            std::abs(p.z() - current_altitude_m) >
                config.z_slice_half_thickness_m) {
          continue;
        }
        iap::RiskVoxel voxel;
        if (!snapshot->voxelAt(horizon_id, id, &voxel)) {
          continue;
        }
        rows.push_back(Row{p, voxel});
      }
    }
  }

  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2Fields(
      34,
      "x", 1, sensor_msgs::msg::PointField::FLOAT32,
      "y", 1, sensor_msgs::msg::PointField::FLOAT32,
      "z", 1, sensor_msgs::msg::PointField::FLOAT32,
      "rgb", 1, sensor_msgs::msg::PointField::FLOAT32,
      "pl", 1, sensor_msgs::msg::PointField::FLOAT32,
      "hpl", 1, sensor_msgs::msg::PointField::FLOAT32,
      "vpl", 1, sensor_msgs::msg::PointField::FLOAT32,
      "c_pi", 1, sensor_msgs::msg::PointField::FLOAT32,
      "risk_ratio", 1, sensor_msgs::msg::PointField::FLOAT32,
      "hal", 1, sensor_msgs::msg::PointField::FLOAT32,
      "val", 1, sensor_msgs::msg::PointField::FLOAT32,
      "gnss_hpl", 1, sensor_msgs::msg::PointField::FLOAT32,
      "gnss_vpl", 1, sensor_msgs::msg::PointField::FLOAT32,
      "gnss_risk_ratio", 1, sensor_msgs::msg::PointField::FLOAT32,
      "lidar_hpl", 1, sensor_msgs::msg::PointField::FLOAT32,
      "lidar_vpl", 1, sensor_msgs::msg::PointField::FLOAT32,
      "lidar_risk_ratio", 1, sensor_msgs::msg::PointField::FLOAT32,
      "prior_hpl", 1, sensor_msgs::msg::PointField::FLOAT32,
      "prior_vpl", 1, sensor_msgs::msg::PointField::FLOAT32,
      "prior_risk_ratio", 1, sensor_msgs::msg::PointField::FLOAT32,
      "fim_hpl", 1, sensor_msgs::msg::PointField::FLOAT32,
      "fim_vpl", 1, sensor_msgs::msg::PointField::FLOAT32,
      "fim_risk_ratio", 1, sensor_msgs::msg::PointField::FLOAT32,
      "floor_increment_h", 1, sensor_msgs::msg::PointField::FLOAT32,
      "floor_increment_v", 1, sensor_msgs::msg::PointField::FLOAT32,
      "floor_source_h", 1, sensor_msgs::msg::PointField::UINT8,
      "floor_source_v", 1, sensor_msgs::msg::PointField::UINT8,
      "valid", 1, sensor_msgs::msg::PointField::UINT8,
      "unknown", 1, sensor_msgs::msg::PointField::UINT8,
      "stale", 1, sensor_msgs::msg::PointField::UINT8,
      "occupied", 1, sensor_msgs::msg::PointField::UINT8,
      "observed", 1, sensor_msgs::msg::PointField::UINT8,
      "source_flags", 1, sensor_msgs::msg::PointField::UINT32,
      "generation_id", 1, sensor_msgs::msg::PointField::UINT32);
  modifier.resize(rows.size());

  sensor_msgs::PointCloud2Iterator<float> x(cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> y(cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> z(cloud, "z");
  sensor_msgs::PointCloud2Iterator<float> rgb(cloud, "rgb");
  sensor_msgs::PointCloud2Iterator<float> pl(cloud, "pl");
  sensor_msgs::PointCloud2Iterator<float> hpl(cloud, "hpl");
  sensor_msgs::PointCloud2Iterator<float> vpl(cloud, "vpl");
  sensor_msgs::PointCloud2Iterator<float> c_pi(cloud, "c_pi");
  sensor_msgs::PointCloud2Iterator<float> risk_ratio(cloud, "risk_ratio");
  sensor_msgs::PointCloud2Iterator<float> hal(cloud, "hal");
  sensor_msgs::PointCloud2Iterator<float> val(cloud, "val");
  sensor_msgs::PointCloud2Iterator<float> gnss_hpl(cloud, "gnss_hpl");
  sensor_msgs::PointCloud2Iterator<float> gnss_vpl(cloud, "gnss_vpl");
  sensor_msgs::PointCloud2Iterator<float> gnss_ratio(cloud, "gnss_risk_ratio");
  sensor_msgs::PointCloud2Iterator<float> lidar_hpl(cloud, "lidar_hpl");
  sensor_msgs::PointCloud2Iterator<float> lidar_vpl(cloud, "lidar_vpl");
  sensor_msgs::PointCloud2Iterator<float> lidar_ratio(cloud, "lidar_risk_ratio");
  sensor_msgs::PointCloud2Iterator<float> prior_hpl(cloud, "prior_hpl");
  sensor_msgs::PointCloud2Iterator<float> prior_vpl(cloud, "prior_vpl");
  sensor_msgs::PointCloud2Iterator<float> prior_ratio(cloud, "prior_risk_ratio");
  sensor_msgs::PointCloud2Iterator<float> fim_hpl(cloud, "fim_hpl");
  sensor_msgs::PointCloud2Iterator<float> fim_vpl(cloud, "fim_vpl");
  sensor_msgs::PointCloud2Iterator<float> fim_ratio(cloud, "fim_risk_ratio");
  sensor_msgs::PointCloud2Iterator<float> floor_h(cloud, "floor_increment_h");
  sensor_msgs::PointCloud2Iterator<float> floor_v(cloud, "floor_increment_v");
  sensor_msgs::PointCloud2Iterator<uint8_t> floor_source_h(
      cloud, "floor_source_h");
  sensor_msgs::PointCloud2Iterator<uint8_t> floor_source_v(
      cloud, "floor_source_v");
  sensor_msgs::PointCloud2Iterator<uint8_t> valid(cloud, "valid");
  sensor_msgs::PointCloud2Iterator<uint8_t> unknown(cloud, "unknown");
  sensor_msgs::PointCloud2Iterator<uint8_t> stale(cloud, "stale");
  sensor_msgs::PointCloud2Iterator<uint8_t> occupied(cloud, "occupied");
  sensor_msgs::PointCloud2Iterator<uint8_t> observed(cloud, "observed");
  sensor_msgs::PointCloud2Iterator<uint32_t> source_flags(cloud,
                                                          "source_flags");
  sensor_msgs::PointCloud2Iterator<uint32_t> generation_id(cloud,
                                                           "generation_id");
  for (const auto& row : rows) {
    const double scalar_pl =
        std::max(row.voxel.hpl_pred, row.voxel.vpl_pred);
    const bool is_occupied =
        (row.voxel.source_flags & iap::RISK_GRID_SOURCE_OCCUPIED_SKIP) != 0u ||
        (row.voxel.occupancy && row.voxel.occupancy->inflated_occupied);
    *x = static_cast<float>(row.p.x());
    *y = static_cast<float>(row.p.y());
    *z = static_cast<float>(row.p.z());
    *rgb = packed_rgb_float(risk_ratio_color(
        row.voxel.risk_ratio, row.voxel.valid, row.voxel.unknown,
        row.voxel.stale, is_occupied));
    *pl = static_cast<float>(scalar_pl);
    *hpl = static_cast<float>(row.voxel.hpl_pred);
    *vpl = static_cast<float>(row.voxel.vpl_pred);
    *c_pi = static_cast<float>(row.voxel.c_pi);
    *risk_ratio = static_cast<float>(row.voxel.risk_ratio);
    *hal = static_cast<float>(row.voxel.hal);
    *val = static_cast<float>(row.voxel.val);
    *gnss_hpl = static_cast<float>(row.voxel.gnss.hpl);
    *gnss_vpl = static_cast<float>(row.voxel.gnss.vpl);
    *gnss_ratio = static_cast<float>(row.voxel.gnss.risk_ratio);
    *lidar_hpl = static_cast<float>(row.voxel.lidar.hpl);
    *lidar_vpl = static_cast<float>(row.voxel.lidar.vpl);
    *lidar_ratio = static_cast<float>(row.voxel.lidar.risk_ratio);
    *prior_hpl = static_cast<float>(row.voxel.prior.hpl);
    *prior_vpl = static_cast<float>(row.voxel.prior.vpl);
    *prior_ratio = static_cast<float>(row.voxel.prior.risk_ratio);
    *fim_hpl = static_cast<float>(row.voxel.fim_fused.hpl);
    *fim_vpl = static_cast<float>(row.voxel.fim_fused.vpl);
    *fim_ratio = static_cast<float>(row.voxel.fim_fused.risk_ratio);
    *floor_h = static_cast<float>(row.voxel.floor_increment_h);
    *floor_v = static_cast<float>(row.voxel.floor_increment_v);
    *floor_source_h = floor_source_code(row.voxel.floor_source_h);
    *floor_source_v = floor_source_code(row.voxel.floor_source_v);
    *valid = row.voxel.valid ? 1u : 0u;
    *unknown = row.voxel.unknown ? 1u : 0u;
    *stale = row.voxel.stale ? 1u : 0u;
    *occupied = is_occupied ? 1u : 0u;
    *observed = row.voxel.occupancy && row.voxel.occupancy->observed ? 1u : 0u;
    *source_flags = row.voxel.source_flags;
    *generation_id = static_cast<uint32_t>(snapshot->generation_id());
    ++x; ++y; ++z; ++rgb; ++pl; ++hpl; ++vpl; ++c_pi;
    ++risk_ratio; ++hal; ++val; ++gnss_hpl; ++gnss_vpl; ++gnss_ratio;
    ++lidar_hpl; ++lidar_vpl; ++lidar_ratio; ++prior_hpl; ++prior_vpl;
    ++prior_ratio; ++fim_hpl; ++fim_vpl; ++fim_ratio; ++floor_h; ++floor_v;
    ++floor_source_h; ++floor_source_v;
    ++valid; ++unknown; ++stale; ++occupied; ++observed; ++source_flags;
    ++generation_id;
  }
  return cloud;
}

sensor_msgs::msg::PointCloud2 SafetyRvizPublisher::buildDiagnosticRiskCloud(
    const std::shared_ptr<const iap::RiskGridSnapshot>& snapshot,
    const Config& config,
    const double current_altitude_m,
    const rclcpp::Time& stamp,
    const std::string& channel,
    const bool relative_scale) {
  auto cloud = buildPredictedPLCloud(
      snapshot, config, current_altitude_m, stamp);
  if (cloud.width == 0u || cloud.height == 0u) {
    return cloud;
  }
  std::string ratio_field = "risk_ratio";
  if (channel == "gnss") {
    ratio_field = "gnss_risk_ratio";
  } else if (channel == "lidar") {
    ratio_field = "lidar_risk_ratio";
  } else if (channel == "fim") {
    ratio_field = "fim_risk_ratio";
  }

  const std::size_t count = static_cast<std::size_t>(cloud.width) *
      static_cast<std::size_t>(cloud.height);
  std::vector<double> ratios;
  ratios.reserve(count);
  sensor_msgs::PointCloud2ConstIterator<float> ratio_read(cloud, ratio_field);
  sensor_msgs::PointCloud2ConstIterator<uint8_t> valid_read(cloud, "valid");
  sensor_msgs::PointCloud2ConstIterator<uint8_t> unknown_read(cloud, "unknown");
  sensor_msgs::PointCloud2ConstIterator<uint8_t> stale_read(cloud, "stale");
  sensor_msgs::PointCloud2ConstIterator<uint8_t> occupied_read(cloud,
                                                               "occupied");
  for (std::size_t i = 0; i < count; ++i) {
    if (*valid_read != 0u && *unknown_read == 0u && *stale_read == 0u &&
        *occupied_read == 0u && finite(*ratio_read)) {
      ratios.push_back(*ratio_read);
    }
    ++ratio_read;
    ++valid_read;
    ++unknown_read;
    ++stale_read;
    ++occupied_read;
  }
  const double p5 = percentile(ratios, 0.05);
  const double p95 = percentile(ratios, 0.95);

  sensor_msgs::PointCloud2Iterator<float> rgb(cloud, "rgb");
  sensor_msgs::PointCloud2ConstIterator<float> ratio(cloud, ratio_field);
  sensor_msgs::PointCloud2ConstIterator<uint8_t> valid(cloud, "valid");
  sensor_msgs::PointCloud2ConstIterator<uint8_t> unknown(cloud, "unknown");
  sensor_msgs::PointCloud2ConstIterator<uint8_t> stale(cloud, "stale");
  sensor_msgs::PointCloud2ConstIterator<uint8_t> occupied(cloud, "occupied");
  for (std::size_t i = 0; i < count; ++i) {
    std_msgs::msg::ColorRGBA point_color;
    if (*occupied != 0u || *stale != 0u || *unknown != 0u ||
        *valid == 0u || !finite(*ratio)) {
      point_color = risk_ratio_color(*ratio, *valid != 0u,
                                     *unknown != 0u || !finite(*ratio),
                                     *stale != 0u, *occupied != 0u);
    } else if (relative_scale) {
      point_color = relative_risk_color(*ratio, p5, p95);
    } else {
      point_color = risk_ratio_color(*ratio, true, false, false, false);
    }
    *rgb = packed_rgb_float(point_color);
    ++rgb;
    ++ratio;
    ++valid;
    ++unknown;
    ++stale;
    ++occupied;
  }
  return cloud;
}

visualization_msgs::msg::MarkerArray
SafetyRvizPublisher::buildPlanningGeometryMarkers(
    const std::shared_ptr<const iap::RiskGridSnapshot>& snapshot,
    const Config& config,
    const double current_altitude_m,
    const rclcpp::Time& stamp) {
  visualization_msgs::msg::MarkerArray markers;
  if (!snapshot) {
    return markers;
  }
  const Eigen::Vector3d origin = snapshot->origin();
  const Eigen::Vector3d extent(snapshot->params().size_x_m,
                               snapshot->params().size_y_m,
                               snapshot->params().size_z_m);
  if (!origin.allFinite() || !extent.allFinite()) {
    return markers;
  }

  auto wireframe = base_marker(config, stamp, "planning_lattice", 0,
      visualization_msgs::msg::Marker::LINE_LIST);
  wireframe.scale.x = 0.06;
  wireframe.color = color(0.1f, 0.85f, 0.95f, 0.8f);
  const std::array<Eigen::Vector3d, 8> corners = {
      origin,
      origin + Eigen::Vector3d(extent.x(), 0.0, 0.0),
      origin + Eigen::Vector3d(extent.x(), extent.y(), 0.0),
      origin + Eigen::Vector3d(0.0, extent.y(), 0.0),
      origin + Eigen::Vector3d(0.0, 0.0, extent.z()),
      origin + Eigen::Vector3d(extent.x(), 0.0, extent.z()),
      origin + extent,
      origin + Eigen::Vector3d(0.0, extent.y(), extent.z())};
  constexpr std::array<std::array<int, 2>, 12> edges = {{
      {{0, 1}}, {{1, 2}}, {{2, 3}}, {{3, 0}},
      {{4, 5}}, {{5, 6}}, {{6, 7}}, {{7, 4}},
      {{0, 4}}, {{1, 5}}, {{2, 6}}, {{3, 7}}}};
  for (const auto& edge : edges) {
    wireframe.points.push_back(point_msg(corners[edge[0]]));
    wireframe.points.push_back(point_msg(corners[edge[1]]));
  }
  markers.markers.push_back(std::move(wireframe));

  auto origin_marker = base_marker(config, stamp, "planning_lattice", 1,
      visualization_msgs::msg::Marker::SPHERE);
  origin_marker.pose.position = point_msg(origin);
  origin_marker.scale.x = 0.35;
  origin_marker.scale.y = 0.35;
  origin_marker.scale.z = 0.35;
  origin_marker.color = color(0.0f, 1.0f, 1.0f, 1.0f);
  markers.markers.push_back(std::move(origin_marker));

  std::vector<double> values;
  const int horizon_id = selected_horizon_index(
      *snapshot, config.selected_horizon_s);
  const bool slice_all = config.z_slice_mode == "all";
  const Eigen::Vector3i dims = snapshot->voxelNum();
  if (horizon_id >= 0) {
    for (int x = 0; x < dims.x(); ++x) {
      for (int y = 0; y < dims.y(); ++y) {
        for (int z = 0; z < dims.z(); ++z) {
          const Eigen::Vector3i index(x, y, z);
          const Eigen::Vector3d position = snapshot->indexToPos(index);
          if (!slice_all && finite(current_altitude_m) &&
              std::abs(position.z() - current_altitude_m) >
                  config.z_slice_half_thickness_m) {
            continue;
          }
          iap::RiskVoxel voxel;
          if (snapshot->voxelAt(horizon_id, index, &voxel) && voxel.valid &&
              !voxel.unknown && !voxel.stale &&
              std::isfinite(voxel.fim_fused.risk_ratio)) {
            values.push_back(voxel.fim_fused.risk_ratio);
          }
        }
      }
    }
  }
  auto legend = base_marker(config, stamp, "planning_lattice", 2,
      visualization_msgs::msg::Marker::TEXT_VIEW_FACING);
  legend.pose.position = point_msg(
      origin + Eigen::Vector3d(0.5, 0.5, extent.z() + 0.5));
  legend.scale.z = 0.35;
  legend.color = color(0.9f, 0.95f, 1.0f, 1.0f);
  double minimum = std::numeric_limits<double>::quiet_NaN();
  double maximum = std::numeric_limits<double>::quiet_NaN();
  if (!values.empty()) {
    const auto range = std::minmax_element(values.begin(), values.end());
    minimum = *range.first;
    maximum = *range.second;
  }
  std::ostringstream label;
  label << "PlanningLattice origin " << fmt_num(origin.x(), 1) << ","
        << fmt_num(origin.y(), 1) << "," << fmt_num(origin.z(), 1)
        << "  extent " << fmt_num(extent.x(), 1) << "x"
        << fmt_num(extent.y(), 1) << "x" << fmt_num(extent.z(), 1)
        << "\ngeometry_id: " << snapshot->params().geometry_id
        << "  generation: " << snapshot->generation_id()
        << "\nrelative FIM ratio P5/P50/P95: "
        << fmt_num(percentile(values, 0.05), 2) << "/"
        << fmt_num(percentile(values, 0.50), 2) << "/"
        << fmt_num(percentile(values, 0.95), 2)
        << "  min/max: " << fmt_num(minimum, 2) << "/"
        << fmt_num(maximum, 2);
  legend.text = label.str();
  markers.markers.push_back(std::move(legend));
  return markers;
}

sensor_msgs::msg::PointCloud2 SafetyRvizPublisher::buildRiskValidityCloud(
    const std::shared_ptr<const iap::RiskGridSnapshot>& snapshot,
    const Config& config,
    double current_altitude_m,
    const rclcpp::Time& stamp) {
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.frame_id = config.frame_id;
  cloud.header.stamp = stamp;
  if (!snapshot) {
    return cloud;
  }
  const int horizon_id =
      selected_horizon_index(*snapshot, config.selected_horizon_s);
  if (horizon_id < 0) {
    return cloud;
  }

  struct Row {
    Eigen::Vector3d p = Eigen::Vector3d::Zero();
    iap::RiskVoxel voxel;
  };
  std::vector<Row> rows;
  rows.reserve(static_cast<std::size_t>(
      std::min(config.max_cloud_points, snapshot->layerVoxelCount())));
  const bool slice_all = config.z_slice_mode == "all";
  const Eigen::Vector3i dims = snapshot->voxelNum();
  for (int x = 0; x < dims.x(); ++x) {
    for (int y = 0; y < dims.y(); ++y) {
      for (int z = 0; z < dims.z(); ++z) {
        if (static_cast<int>(rows.size()) >= config.max_cloud_points) {
          break;
        }
        const Eigen::Vector3i id(x, y, z);
        const Eigen::Vector3d p = snapshot->indexToPos(id);
        if (!slice_all && finite(current_altitude_m) &&
            std::abs(p.z() - current_altitude_m) >
                config.z_slice_half_thickness_m) {
          continue;
        }
        iap::RiskVoxel voxel;
        if (snapshot->voxelAt(horizon_id, id, &voxel)) {
          rows.push_back(Row{p, voxel});
        }
      }
    }
  }

  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2Fields(
      9,
      "x", 1, sensor_msgs::msg::PointField::FLOAT32,
      "y", 1, sensor_msgs::msg::PointField::FLOAT32,
      "z", 1, sensor_msgs::msg::PointField::FLOAT32,
      "rgb", 1, sensor_msgs::msg::PointField::FLOAT32,
      "valid", 1, sensor_msgs::msg::PointField::UINT8,
      "unknown", 1, sensor_msgs::msg::PointField::UINT8,
      "stale", 1, sensor_msgs::msg::PointField::UINT8,
      "source_flags", 1, sensor_msgs::msg::PointField::UINT32,
      "generation_id", 1, sensor_msgs::msg::PointField::UINT32);
  modifier.resize(rows.size());

  sensor_msgs::PointCloud2Iterator<float> x(cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> y(cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> z(cloud, "z");
  sensor_msgs::PointCloud2Iterator<float> rgb(cloud, "rgb");
  sensor_msgs::PointCloud2Iterator<uint8_t> valid(cloud, "valid");
  sensor_msgs::PointCloud2Iterator<uint8_t> unknown(cloud, "unknown");
  sensor_msgs::PointCloud2Iterator<uint8_t> stale(cloud, "stale");
  sensor_msgs::PointCloud2Iterator<uint32_t> source_flags(cloud,
                                                          "source_flags");
  sensor_msgs::PointCloud2Iterator<uint32_t> generation_id(cloud,
                                                           "generation_id");
  for (const auto& row : rows) {
    *x = static_cast<float>(row.p.x());
    *y = static_cast<float>(row.p.y());
    *z = static_cast<float>(row.p.z());
    *rgb = packed_rgb_float(validity_color(row.voxel));
    *valid = row.voxel.valid ? 1u : 0u;
    *unknown = row.voxel.unknown ? 1u : 0u;
    *stale = row.voxel.stale ? 1u : 0u;
    *source_flags = row.voxel.source_flags;
    *generation_id = static_cast<uint32_t>(snapshot->generation_id());
    ++x; ++y; ++z; ++rgb; ++valid; ++unknown; ++stale; ++source_flags;
    ++generation_id;
  }
  return cloud;
}

visualization_msgs::msg::MarkerArray
SafetyRvizPublisher::buildTrajectorySampleMarkers(
    const SafetyVizGateStatus& status,
    const Config& config,
    const rclcpp::Time& stamp) {
  visualization_msgs::msg::MarkerArray arr;
  auto del = base_marker(config, stamp, "trajectory_integrity_samples", 0,
                         visualization_msgs::msg::Marker::SPHERE);
  del.action = visualization_msgs::msg::Marker::DELETEALL;
  arr.markers.push_back(del);

  int id = 1;
  for (const auto& sample : status.samples) {
    auto marker = base_marker(config, stamp, "trajectory_integrity_samples",
                              id++, visualization_msgs::msg::Marker::SPHERE);
    marker.pose.position = point_msg(sample.position);
    const double severity =
        finite(sample.im_min) ? std::max(0.0, -sample.im_min) : 0.0;
    const double radius = 0.16 + std::min(0.35, 0.08 * severity);
    marker.scale.x = radius;
    marker.scale.y = radius;
    marker.scale.z = radius;
    marker.color = sample_color(sample);
    arr.markers.push_back(marker);
  }

  auto first_bad = std::find_if(
      status.samples.begin(), status.samples.end(),
      [](const SafetyVizTrajectorySample& s) { return s.bad; });
  if (first_bad != status.samples.end()) {
    auto label = base_marker(config, stamp, "p5_first_bad_label", 10000,
                             visualization_msgs::msg::Marker::TEXT_VIEW_FACING);
    label.pose.position = point_msg(first_bad->position);
    label.pose.position.z += 0.55;
    label.scale.z = 0.28;
    label.color = color(1.0f, 0.1f, 0.05f, 1.0f);
    label.text = "first_bad_tau: " + fmt_num(first_bad->tau_s, 2) +
                 " s\nIM: " + fmt_num(first_bad->im_min, 2) +
                 " m\nreason: " + first_bad->reason;
    arr.markers.push_back(label);
  }
  return arr;
}

visualization_msgs::msg::MarkerArray
SafetyRvizPublisher::buildTrajectoryLineMarkers(
    const SafetyVizGateStatus& status,
    const Config& config,
    const rclcpp::Time& stamp) {
  visualization_msgs::msg::MarkerArray arr;
  auto del = base_marker(config, stamp, "current_traj_integrity", 0,
                         visualization_msgs::msg::Marker::LINE_LIST);
  del.action = visualization_msgs::msg::Marker::DELETEALL;
  arr.markers.push_back(del);
  if (status.samples.size() < 2) {
    return arr;
  }

  auto line = base_marker(config, stamp, "current_traj_integrity", 1,
                          visualization_msgs::msg::Marker::LINE_LIST);
  line.scale.x = 0.08;
  for (std::size_t i = 1; i < status.samples.size(); ++i) {
    const auto& a = status.samples[i - 1];
    const auto& b = status.samples[i];
    line.points.push_back(point_msg(a.position));
    line.points.push_back(point_msg(b.position));
    line.colors.push_back(sample_color(a));
    line.colors.push_back(sample_color(b));
  }
  arr.markers.push_back(line);
  return arr;
}

visualization_msgs::msg::MarkerArray
SafetyRvizPublisher::buildP5GateStatusMarkers(
    const SafetyVizGateStatus& status,
    const Config& config,
    const rclcpp::Time& stamp) {
  visualization_msgs::msg::MarkerArray arr;
  auto marker = base_marker(config, stamp, "p5_gate_status", 0,
                            visualization_msgs::msg::Marker::TEXT_VIEW_FACING);
  Eigen::Vector3d pos(15.0, 0.0, 4.5);
  if (!status.samples.empty()) {
    pos = status.samples.front().position + Eigen::Vector3d(0.0, 0.0, 1.0);
  }
  marker.pose.position = point_msg(pos);
  marker.scale.z = 0.36;
  marker.color = action_color(status.action);
  std::ostringstream text;
  text << "P5(" << status.phase << "): " << status.action << "\n"
       << "reason: " << status.reason << "\n"
       << "future_min_IM: " << fmt_num(status.future_min_im, 2) << " m\n"
       << "first_bad_tau: " << fmt_num(status.first_bad_tau, 2) << " s\n"
       << "bad: " << fmt_num(100.0 * status.bad_ratio, 1)
       << "% unknown: " << fmt_num(100.0 * status.unknown_ratio, 1) << "%";
  marker.text = text.str();
  arr.markers.push_back(marker);
  return arr;
}

visualization_msgs::msg::MarkerArray
SafetyRvizPublisher::buildP5CurrentImBarMarkers(
    const SafetyVizGateStatus& status,
    const Config& config,
    const rclcpp::Time& stamp) {
  visualization_msgs::msg::MarkerArray arr;
  auto del = base_marker(config, stamp, "p5_current_im_bars", 0,
                         visualization_msgs::msg::Marker::CUBE);
  del.action = visualization_msgs::msg::Marker::DELETEALL;
  arr.markers.push_back(del);

  Eigen::Vector3d base(15.0, 2.0, 1.0);
  if (!status.samples.empty()) {
    base = status.samples.front().position + Eigen::Vector3d(0.6, 0.0, 0.25);
  }
  const double margins[2] = {status.current_im_h, status.current_im_v};
  const char* labels[2] = {"H-IM HAL-HPL", "V-IM VAL-VPL"};
  for (int i = 0; i < 2; ++i) {
    const double margin = margins[i];
    const double height =
        finite(margin) ? std::clamp(std::abs(margin), 0.08, 2.0) : 0.12;
    auto bar = base_marker(config, stamp, "p5_current_im_bars", 1 + i,
                           visualization_msgs::msg::Marker::CUBE);
    bar.pose.position = point_msg(base + Eigen::Vector3d(0.0, 0.35 * i, 0.0));
    bar.pose.position.z += margin >= 0.0 || !finite(margin)
                               ? 0.5 * height
                               : -0.5 * height;
    bar.scale.x = 0.18;
    bar.scale.y = 0.18;
    bar.scale.z = height;
    bar.color = margin_color(margin);
    arr.markers.push_back(bar);

    auto text = base_marker(config, stamp, "p5_current_im_bars", 10 + i,
                            visualization_msgs::msg::Marker::TEXT_VIEW_FACING);
    text.pose.position = point_msg(base + Eigen::Vector3d(0.35, 0.35 * i, 0.35));
    text.scale.z = 0.22;
    text.color = margin_color(margin);
    std::ostringstream oss;
    oss << labels[i] << ": " << fmt_num(margin, 2) << " m";
    text.text = oss.str();
    arr.markers.push_back(text);
  }
  return arr;
}

visualization_msgs::msg::MarkerArray
SafetyRvizPublisher::buildP1IntegritySampleMarkers(
    const std::vector<SafetyVizP1Sample>& samples,
    const Config& config,
    const rclcpp::Time& stamp) {
  visualization_msgs::msg::MarkerArray arr;
  auto del = base_marker(config, stamp, "p1_integrity_samples", 0,
                         visualization_msgs::msg::Marker::SPHERE);
  del.action = visualization_msgs::msg::Marker::DELETEALL;
  arr.markers.push_back(del);
  int id = 1;
  for (const auto& sample : samples) {
    auto marker = base_marker(config, stamp, "p1_integrity_samples", id++,
                              visualization_msgs::msg::Marker::SPHERE);
    marker.pose.position = point_msg(sample.position);
    marker.scale.x = marker.scale.y = marker.scale.z = 0.14;
    if (sample.stale) {
      marker.color = color(1.0f, 0.75f, 0.05f, 0.9f);
    } else if (sample.unknown || !sample.hit) {
      marker.color = color(0.55f, 0.55f, 0.55f, 0.85f);
    } else {
      const float heat =
          static_cast<float>(std::clamp(sample.cost / 10.0, 0.0, 1.0));
      marker.color = color(heat, 0.85f * (1.0f - heat), 1.0f - heat, 0.9f);
    }
    arr.markers.push_back(marker);
  }
  return arr;
}

visualization_msgs::msg::MarkerArray
SafetyRvizPublisher::buildP1PushVectorMarkers(
    const std::vector<SafetyVizP1Sample>& samples,
    const Config& config,
    const rclcpp::Time& stamp) {
  visualization_msgs::msg::MarkerArray arr;
  auto del = base_marker(config, stamp, "p1_integrity_push_vectors", 0,
                         visualization_msgs::msg::Marker::ARROW);
  del.action = visualization_msgs::msg::Marker::DELETEALL;
  arr.markers.push_back(del);
  int id = 1;
  for (const auto& sample : samples) {
    if (!sample.hit || !sample.push.allFinite() || sample.push.norm() <= 1.0e-6) {
      continue;
    }
    auto arrow = base_marker(config, stamp, "p1_integrity_push_vectors", id++,
                             visualization_msgs::msg::Marker::ARROW);
    arrow.points.push_back(point_msg(sample.position));
    arrow.points.push_back(point_msg(sample.position + sample.push));
    arrow.scale.x = 0.04;
    arrow.scale.y = 0.08;
    arrow.scale.z = 0.12;
    arrow.color = color(0.95f, 0.2f, 0.8f, 0.95f);
    arr.markers.push_back(arrow);
  }
  return arr;
}

visualization_msgs::msg::MarkerArray
SafetyRvizPublisher::buildP1MetricsMarkers(
    const SafetyVizP1Metrics& metrics,
    const Config& config,
    const rclcpp::Time& stamp) {
  visualization_msgs::msg::MarkerArray arr;
  auto marker = base_marker(config, stamp, "p1_integrity_metrics", 0,
                            visualization_msgs::msg::Marker::TEXT_VIEW_FACING);
  marker.pose.position.x = 15.0;
  marker.pose.position.y = 5.0;
  marker.pose.position.z = 4.5;
  marker.scale.z = 0.32;
  marker.color = metrics.applied_to_objective
                     ? color(0.1f, 0.85f, 0.25f, 1.0f)
                     : color(0.0f, 0.75f, 0.95f, 1.0f);
  std::ostringstream text;
  text << "P1 integrity\n"
       << "samples: " << metrics.sample_count
       << " hit: " << metrics.hit_count
       << " miss: " << metrics.miss_count
       << " stale: " << metrics.stale_count << "\n"
       << "f: " << fmt_num(metrics.f_integrity, 3)
       << " weighted: " << fmt_num(metrics.weighted_f_integrity, 3) << "\n"
       << "grad_ratio: " << fmt_num(metrics.grad_ratio, 3)
       << " applied: " << (metrics.applied_to_objective ? "yes" : "no")
       << "\nreason: " << metrics.fallback_reason;
  marker.text = text.str();
  arr.markers.push_back(marker);
  return arr;
}

visualization_msgs::msg::MarkerArray
SafetyRvizPublisher::buildP2CandidateMarkers(
    const std::vector<SafetyVizP2Candidate>& candidates,
    const Config& config,
    const rclcpp::Time& stamp) {
  visualization_msgs::msg::MarkerArray arr;
  auto del = base_marker(config, stamp, "p2_candidate_trajectories", 0,
                         visualization_msgs::msg::Marker::LINE_STRIP);
  del.action = visualization_msgs::msg::Marker::DELETEALL;
  arr.markers.push_back(del);
  int id = 1;
  for (const auto& candidate : candidates) {
    auto line = base_marker(config, stamp, "p2_candidate_trajectories", id++,
                            visualization_msgs::msg::Marker::LINE_STRIP);
    line.scale.x = candidate.selected ? 0.10 : 0.04;
    line.color = candidate.selected ? color(0.1f, 0.85f, 0.25f, 1.0f)
                                    : color(0.55f, 0.55f, 0.55f, 0.45f);
    for (const auto& p : candidate.control_points) {
      line.points.push_back(point_msg(p));
    }
    arr.markers.push_back(line);
    if (!candidate.control_points.empty()) {
      auto label = base_marker(config, stamp, "p2_candidate_trajectories",
                               id++,
                               visualization_msgs::msg::Marker::TEXT_VIEW_FACING);
      label.pose.position = point_msg(candidate.control_points.back() +
                                      Eigen::Vector3d(0.0, 0.0, 0.4));
      label.scale.z = 0.22;
      label.color = line.color;
      label.text = "P2 #" + std::to_string(candidate.candidate_id) +
                   (candidate.selected ? " selected" : " rejected") +
                   "\nscore: " + fmt_num(candidate.score, 2) +
                   " valid: " + fmt_num(100.0 * candidate.valid_ratio, 0) +
                   "%\n" + candidate.reason;
      arr.markers.push_back(label);
    }
  }
  return arr;
}

visualization_msgs::msg::MarkerArray
SafetyRvizPublisher::buildP3ReferenceBiasMarkers(
    const SafetyVizP3ReferenceBias& bias,
    const Config& config,
    const rclcpp::Time& stamp) {
  visualization_msgs::msg::MarkerArray arr;
  auto del = base_marker(config, stamp, "p3_reference_bias", 0,
                         visualization_msgs::msg::Marker::LINE_LIST);
  del.action = visualization_msgs::msg::Marker::DELETEALL;
  arr.markers.push_back(del);

  auto line = base_marker(config, stamp, "p3_reference_bias", 1,
                          visualization_msgs::msg::Marker::LINE_LIST);
  line.scale.x = 0.07;
  line.points.push_back(point_msg(bias.start));
  line.points.push_back(point_msg(bias.nominal_target));
  line.colors.push_back(color(0.2f, 0.45f, 1.0f, 0.75f));
  line.colors.push_back(color(0.2f, 0.45f, 1.0f, 0.75f));
  if (bias.local) {
    line.points.push_back(point_msg(bias.start));
    line.points.push_back(point_msg(bias.biased_target));
    line.colors.push_back(color(0.1f, 0.85f, 0.25f, 0.95f));
    line.colors.push_back(color(0.1f, 0.85f, 0.25f, 0.95f));
  } else {
    Eigen::Vector3d prev = bias.start;
    for (const auto& p : bias.biased_waypoints) {
      line.points.push_back(point_msg(prev));
      line.points.push_back(point_msg(p));
      line.colors.push_back(color(0.1f, 0.85f, 0.25f, 0.95f));
      line.colors.push_back(color(0.1f, 0.85f, 0.25f, 0.95f));
      prev = p;
    }
    line.points.push_back(point_msg(prev));
    line.points.push_back(point_msg(bias.end));
    line.colors.push_back(color(0.1f, 0.85f, 0.25f, 0.95f));
    line.colors.push_back(color(0.1f, 0.85f, 0.25f, 0.95f));
  }
  arr.markers.push_back(line);

  auto label = base_marker(config, stamp, "p3_reference_bias", 2,
                           visualization_msgs::msg::Marker::TEXT_VIEW_FACING);
  label.pose.position = point_msg((bias.start + bias.end) * 0.5 +
                                  Eigen::Vector3d(0.0, 0.0, 0.7));
  label.scale.z = 0.24;
  label.color = bias.used_bias ? color(0.1f, 0.85f, 0.25f, 1.0f)
                               : color(0.55f, 0.55f, 0.55f, 0.9f);
  label.text = std::string("P3 ") + (bias.local ? "local" : "global") +
               (bias.used_bias ? " used" : " nominal") +
               "\nimprovement: " + fmt_num(bias.improvement_ratio, 3) +
               "\nreason: " + bias.reason;
  arr.markers.push_back(label);
  return arr;
}

visualization_msgs::msg::MarkerArray
SafetyRvizPublisher::buildP4GuideMarkers(
    const std::vector<SafetyVizP4Guide>& guides,
    const Config& config,
    const rclcpp::Time& stamp) {
  visualization_msgs::msg::MarkerArray arr;
  auto del = base_marker(config, stamp, "p4_astar_guides", 0,
                         visualization_msgs::msg::Marker::LINE_STRIP);
  del.action = visualization_msgs::msg::Marker::DELETEALL;
  arr.markers.push_back(del);
  int id = 1;
  auto add_path = [&](const std::vector<Eigen::Vector3d>& path,
                      const std_msgs::msg::ColorRGBA& c,
                      double width) {
    if (path.size() < 2) {
      return;
    }
    auto line = base_marker(config, stamp, "p4_astar_guides", id++,
                            visualization_msgs::msg::Marker::LINE_STRIP);
    line.scale.x = width;
    line.color = c;
    for (const auto& p : path) {
      line.points.push_back(point_msg(p));
    }
    arr.markers.push_back(line);
  };
  for (const auto& guide : guides) {
    if (guide.forward_decision) {
      for (const auto& raw : guide.raw_topology_paths) {
        add_path(raw, color(0.7f, 0.7f, 0.75f, 0.22f), 0.018);
      }
      for (std::size_t index = 0;
           index < guide.topology_candidates.size(); ++index) {
        const bool supported =
            index < guide.topology_candidate_supported.size() &&
            guide.topology_candidate_supported[index];
        add_path(
            guide.topology_candidates[index],
            supported ? color(0.3f, 0.65f, 1.0f, 0.65f)
                      : color(0.75f, 0.2f, 0.85f, 0.75f),
            0.045);
        if (!guide.topology_candidates[index].empty() &&
            index < guide.topology_candidate_labels.size()) {
          auto label = base_marker(
              config, stamp, "p4_astar_guides", id++,
              visualization_msgs::msg::Marker::TEXT_VIEW_FACING);
          label.pose.position = point_msg(
              guide.topology_candidates[index][
                  guide.topology_candidates[index].size() / 2]);
          label.pose.position.z += 0.22;
          label.scale.z = 0.18;
          label.color = supported ? color(0.8f, 0.9f, 1.0f, 0.9f)
                                  : color(1.0f, 0.45f, 1.0f, 0.95f);
          label.text = guide.topology_candidate_labels[index];
          arr.markers.push_back(std::move(label));
        }
      }
      add_path(guide.observe_more_path,
               color(0.1f, 0.95f, 0.95f, 0.95f), 0.09);
    }
    add_path(guide.original_path, color(0.2f, 0.45f, 1.0f, 0.45f), 0.04);
    add_path(guide.risk_path, color(1.0f, 0.52f, 0.04f, 0.65f), 0.06);
    add_path(guide.selected_path,
             guide.risk_selected ? color(0.1f, 0.85f, 0.25f, 1.0f)
                                 : color(0.9f, 0.9f, 0.1f, 0.85f),
             0.10);
    auto endpoints = base_marker(config, stamp, "p4_astar_guides", id++,
        visualization_msgs::msg::Marker::SPHERE_LIST);
    endpoints.scale.x = endpoints.scale.y = endpoints.scale.z = 0.22;
    endpoints.color = color(0.95f, 0.9f, 0.15f, 1.0f);
    endpoints.points.push_back(point_msg(guide.segment_start));
    endpoints.points.push_back(point_msg(guide.segment_end));
    arr.markers.push_back(std::move(endpoints));
    if (guide.forward_decision && guide.common_anchor.allFinite()) {
      auto anchor = base_marker(config, stamp, "p4_astar_guides", id++,
          visualization_msgs::msg::Marker::SPHERE);
      anchor.pose.position = point_msg(guide.common_anchor);
      anchor.scale.x = anchor.scale.y = anchor.scale.z = 0.34;
      anchor.color = color(0.15f, 0.95f, 0.9f, 0.95f);
      arr.markers.push_back(std::move(anchor));
      const auto add_ring = [&](const double radius,
                                const std_msgs::msg::ColorRGBA& ring_color) {
          if (!guide.uav_position.allFinite() || radius <= 0.0) return;
          auto ring = base_marker(config, stamp, "p4_astar_guides", id++,
              visualization_msgs::msg::Marker::LINE_STRIP);
          ring.scale.x = 0.025;
          ring.color = ring_color;
          constexpr int kRingSegments = 48;
          for (int segment = 0; segment <= kRingSegments; ++segment) {
            const double angle = 2.0 * M_PI * segment / kRingSegments;
            ring.points.push_back(point_msg(
                guide.uav_position + Eigen::Vector3d(
                    radius * std::cos(angle), radius * std::sin(angle), 0.0)));
          }
          arr.markers.push_back(std::move(ring));
        };
      add_ring(guide.decision_horizon_m, color(0.2f, 0.7f, 1.0f, 0.65f));
      add_ring(guide.stopping_distance_m, color(1.0f, 0.25f, 0.1f, 0.8f));
    }
    if (guide.uav_position.allFinite()) {
      auto connector = base_marker(config, stamp, "p4_astar_guides", id++,
          visualization_msgs::msg::Marker::LINE_LIST);
      connector.scale.x = 0.025;
      connector.color = color(0.75f, 0.75f, 0.75f, 0.65f);
      const Eigen::Vector3d delta = guide.segment_start - guide.uav_position;
      constexpr int kDashCount = 12;
      for (int dash = 0; dash < kDashCount; dash += 2) {
        const double begin = static_cast<double>(dash) / kDashCount;
        const double end = static_cast<double>(dash + 1) / kDashCount;
        connector.points.push_back(point_msg(
            guide.uav_position + begin * delta));
        connector.points.push_back(point_msg(
            guide.uav_position + end * delta));
      }
      arr.markers.push_back(std::move(connector));
    }
    if (guide.forward_decision && guide.first_failed_position.allFinite()) {
      auto failed = base_marker(config, stamp, "p4_astar_guides", id++,
          visualization_msgs::msg::Marker::SPHERE);
      failed.pose.position = point_msg(guide.first_failed_position);
      failed.scale.x = failed.scale.y = failed.scale.z = 0.28;
      failed.color = color(1.0f, 0.1f, 0.05f, 0.9f);
      arr.markers.push_back(std::move(failed));

      auto failed_label = base_marker(
          config, stamp, "p4_astar_guides", id++,
          visualization_msgs::msg::Marker::TEXT_VIEW_FACING);
      failed_label.pose.position = point_msg(
          guide.first_failed_position + Eigen::Vector3d(0.0, 0.0, 0.4));
      failed_label.scale.z = 0.18;
      failed_label.color = color(1.0f, 0.3f, 0.15f, 1.0f);
      failed_label.text = "P4 first failure: " + guide.first_failed_reason +
          "\ncandidate=" + std::to_string(
              guide.first_failed_candidate_id) +
          " arc=" + fmt_num(guide.first_failed_arc_length_m, 2) + " m" +
          "\nknown=" + std::to_string(guide.first_failed_gnss_known_count) +
          " visible=" + std::to_string(
              guide.first_failed_gnss_visible_count) +
          " blocked=" + std::to_string(
              guide.first_failed_gnss_blocked_count) +
          " unknown=" + std::to_string(
              guide.first_failed_gnss_unknown_count) +
          " used=" + std::to_string(guide.first_failed_gnss_used_count) +
          "\nHPL " + fmt_num(guide.first_failed_hpl, 1) + "/" +
          fmt_num(guide.first_failed_hal, 1) +
          " VPL " + fmt_num(guide.first_failed_vpl, 1) + "/" +
          fmt_num(guide.first_failed_val, 1) +
          "\nfloor=" + guide.first_failed_floor_source_h + "/" +
          guide.first_failed_floor_source_v +
          " tau=" + fmt_num(
              guide.first_failed_query_time_s - guide.risk_snapshot_stamp_s,
              1);
      arr.markers.push_back(std::move(failed_label));
    }
    auto label = base_marker(config, stamp, "p4_astar_guides", id++,
                             visualization_msgs::msg::Marker::TEXT_VIEW_FACING);
    label.pose.position = point_msg((guide.segment_start + guide.segment_end) *
                                    0.5 + Eigen::Vector3d(0.0, 0.0, 0.6));
    label.scale.z = 0.22;
    label.color = guide.risk_selected ? color(0.1f, 0.85f, 0.25f, 1.0f)
                                      : color(1.0f, 0.75f, 0.05f, 1.0f);
    label.text = std::string(guide.forward_decision ?
                 "P4 forward route decision\n" :
                 "P4 local collision guide\n") +
                 (guide.risk_selected ? "risk guide" : "original guide") +
                 "\nratio: " + fmt_num(guide.path_length_ratio, 2) +
                 "\nevidence age: " + fmt_num(
                     stamp.seconds() - guide.risk_snapshot_stamp_s, 2) +
                 " s" +
                 "\nreason: " + guide.reason;
    arr.markers.push_back(label);
  }
  return arr;
}

visualization_msgs::msg::MarkerArray
SafetyRvizPublisher::buildP4TopologyChannelMarkers(
    const std::vector<SafetyVizP4Guide>& guides,
    const Config& config,
    const rclcpp::Time& stamp) {
  visualization_msgs::msg::MarkerArray arr;
  constexpr char kNamespace[] = "p4_topology_channels";

  const std::array<std_msgs::msg::ColorRGBA, 4> channel_colors = {
      color(0.05f, 0.85f, 1.0f, 0.92f),
      color(1.0f, 0.25f, 0.75f, 0.92f),
      color(1.0f, 0.62f, 0.08f, 0.92f),
      color(0.25f, 0.95f, 0.35f, 0.92f)};
  const auto marker_lifetime = rclcpp::Duration::from_seconds(
      std::max(1.5, 2.0 / config.publish_rate_hz));
  for (const auto& guide : guides) {
    if (!guide.forward_decision) {
      continue;
    }
    for (std::size_t index = 0;
         index < guide.topology_candidates.size(); ++index) {
      const auto& path = guide.topology_candidates[index];
      if (path.size() < 2) {
        continue;
      }
      const uint64_t channel_id =
          index < guide.topology_channel_ids.size() &&
          guide.topology_channel_ids[index] > 0
          ? guide.topology_channel_ids[index]
          : static_cast<uint64_t>(index + 1);
      const auto& channel_color = channel_colors[
          static_cast<std::size_t>((channel_id - 1) % channel_colors.size())];
      const int marker_base = p4_channel_marker_base(channel_id);
      auto line = base_marker(config, stamp, kNamespace, marker_base,
                              visualization_msgs::msg::Marker::LINE_STRIP);
      line.scale.x = 0.12;
      line.color = channel_color;
      line.lifetime = marker_lifetime;
      for (const auto& point : path) {
        line.points.push_back(point_msg(point));
      }
      arr.markers.push_back(std::move(line));

      auto label = base_marker(
          config, stamp, kNamespace, marker_base + 1,
          visualization_msgs::msg::Marker::TEXT_VIEW_FACING);
      label.pose.position = point_msg(path[path.size() / 2]);
      label.pose.position.z += 0.35;
      label.scale.z = 0.22;
      label.color = channel_color;
      label.lifetime = marker_lifetime;
      label.text = "P4 channel " + std::to_string(channel_id);
      if (index < guide.topology_candidate_labels.size() &&
          !guide.topology_candidate_labels[index].empty()) {
        label.text += "\n" + guide.topology_candidate_labels[index];
      }
      arr.markers.push_back(std::move(label));
    }

    if (guide.observe_more_path.size() >= 2) {
      auto prefix = base_marker(
          config, stamp, kNamespace, 1,
          visualization_msgs::msg::Marker::LINE_STRIP);
      prefix.scale.x = 0.16;
      prefix.color = color(1.0f, 1.0f, 1.0f, 1.0f);
      prefix.lifetime = marker_lifetime;
      for (const auto& point : guide.observe_more_path) {
        prefix.points.push_back(point_msg(point));
      }
      arr.markers.push_back(std::move(prefix));
    }
  }
  return arr;
}

}  // namespace ego_planner
