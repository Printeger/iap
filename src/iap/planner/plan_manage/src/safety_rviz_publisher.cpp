#include <ego_planner/safety_rviz_publisher.h>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <iomanip>
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

std_msgs::msg::ColorRGBA pl_color(double pl,
                                  bool valid,
                                  bool unknown,
                                  bool stale) {
  if (stale) {
    return color(0.2f, 0.2f, 0.2f, 0.55f);
  }
  if (unknown || !valid || !finite(pl)) {
    return color(0.55f, 0.55f, 0.55f, 0.65f);
  }
  if (pl < 1.0) {
    return color(0.1f, 0.45f, 1.0f, 0.9f);
  }
  if (pl < 2.0) {
    return color(0.0f, 0.9f, 0.9f, 0.9f);
  }
  if (pl < 4.0) {
    return color(0.1f, 0.85f, 0.25f, 0.9f);
  }
  if (pl < 8.0) {
    return color(1.0f, 0.8f, 0.05f, 0.95f);
  }
  if (pl < 15.0) {
    return color(1.0f, 0.42f, 0.02f, 0.95f);
  }
  return color(1.0f, 0.02f, 0.02f, 1.0f);
}

std_msgs::msg::ColorRGBA validity_color(const iap::RiskVoxel& voxel) {
  if (voxel.stale) {
    return color(0.12f, 0.12f, 0.12f, 0.8f);
  }
  if (voxel.unknown || !voxel.valid) {
    return color(0.55f, 0.55f, 0.55f, 0.75f);
  }
  if ((voxel.source_flags & iap::RISK_GRID_SOURCE_OCCUPIED_SKIP) != 0u) {
    return color(1.0f, 0.9f, 0.05f, 0.9f);
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
    double now_s) {
  if (!predicted_pl_cloud_pub_ || !shouldPublish(now_s, &last_grid_publish_s_)) {
    return;
  }
  const rclcpp::Time stamp = stamp_from_seconds(node_, now_s);
  predicted_pl_cloud_pub_->publish(
      buildPredictedPLCloud(snapshot, config_, current_altitude_m, stamp));
}

void SafetyRvizPublisher::publishRiskValidityCloud(
    const std::shared_ptr<const iap::RiskGridSnapshot>& snapshot,
    double current_altitude_m,
    double now_s) {
  if (!risk_validity_cloud_pub_ ||
      !shouldPublish(now_s, &last_validity_publish_s_)) {
    return;
  }
  const rclcpp::Time stamp = stamp_from_seconds(node_, now_s);
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
  if (!p4_astar_guides_pub_ || !shouldPublish(now_s, &last_p4_publish_s_)) {
    return;
  }
  const rclcpp::Time stamp = stamp_from_seconds(node_, now_s);
  p4_astar_guides_pub_->publish(buildP4GuideMarkers(guides, config_, stamp));
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
      12,
      "x", 1, sensor_msgs::msg::PointField::FLOAT32,
      "y", 1, sensor_msgs::msg::PointField::FLOAT32,
      "z", 1, sensor_msgs::msg::PointField::FLOAT32,
      "rgb", 1, sensor_msgs::msg::PointField::FLOAT32,
      "pl", 1, sensor_msgs::msg::PointField::FLOAT32,
      "hpl", 1, sensor_msgs::msg::PointField::FLOAT32,
      "vpl", 1, sensor_msgs::msg::PointField::FLOAT32,
      "c_pi", 1, sensor_msgs::msg::PointField::FLOAT32,
      "valid", 1, sensor_msgs::msg::PointField::UINT8,
      "unknown", 1, sensor_msgs::msg::PointField::UINT8,
      "stale", 1, sensor_msgs::msg::PointField::UINT8,
      "source_flags", 1, sensor_msgs::msg::PointField::UINT32);
  modifier.resize(rows.size());

  sensor_msgs::PointCloud2Iterator<float> x(cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> y(cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> z(cloud, "z");
  sensor_msgs::PointCloud2Iterator<float> rgb(cloud, "rgb");
  sensor_msgs::PointCloud2Iterator<float> pl(cloud, "pl");
  sensor_msgs::PointCloud2Iterator<float> hpl(cloud, "hpl");
  sensor_msgs::PointCloud2Iterator<float> vpl(cloud, "vpl");
  sensor_msgs::PointCloud2Iterator<float> c_pi(cloud, "c_pi");
  sensor_msgs::PointCloud2Iterator<uint8_t> valid(cloud, "valid");
  sensor_msgs::PointCloud2Iterator<uint8_t> unknown(cloud, "unknown");
  sensor_msgs::PointCloud2Iterator<uint8_t> stale(cloud, "stale");
  sensor_msgs::PointCloud2Iterator<uint32_t> source_flags(cloud,
                                                          "source_flags");
  for (const auto& row : rows) {
    const double scalar_pl =
        std::max(row.voxel.hpl_pred, row.voxel.vpl_pred);
    *x = static_cast<float>(row.p.x());
    *y = static_cast<float>(row.p.y());
    *z = static_cast<float>(row.p.z());
    *rgb = packed_rgb_float(pl_color(scalar_pl, row.voxel.valid,
                                     row.voxel.unknown, row.voxel.stale));
    *pl = static_cast<float>(scalar_pl);
    *hpl = static_cast<float>(row.voxel.hpl_pred);
    *vpl = static_cast<float>(row.voxel.vpl_pred);
    *c_pi = static_cast<float>(row.voxel.c_pi);
    *valid = row.voxel.valid ? 1u : 0u;
    *unknown = row.voxel.unknown ? 1u : 0u;
    *stale = row.voxel.stale ? 1u : 0u;
    *source_flags = row.voxel.source_flags;
    ++x; ++y; ++z; ++rgb; ++pl; ++hpl; ++vpl; ++c_pi;
    ++valid; ++unknown; ++stale; ++source_flags;
  }
  return cloud;
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
      8,
      "x", 1, sensor_msgs::msg::PointField::FLOAT32,
      "y", 1, sensor_msgs::msg::PointField::FLOAT32,
      "z", 1, sensor_msgs::msg::PointField::FLOAT32,
      "rgb", 1, sensor_msgs::msg::PointField::FLOAT32,
      "valid", 1, sensor_msgs::msg::PointField::UINT8,
      "unknown", 1, sensor_msgs::msg::PointField::UINT8,
      "stale", 1, sensor_msgs::msg::PointField::UINT8,
      "source_flags", 1, sensor_msgs::msg::PointField::UINT32);
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
  for (const auto& row : rows) {
    *x = static_cast<float>(row.p.x());
    *y = static_cast<float>(row.p.y());
    *z = static_cast<float>(row.p.z());
    *rgb = packed_rgb_float(validity_color(row.voxel));
    *valid = row.voxel.valid ? 1u : 0u;
    *unknown = row.voxel.unknown ? 1u : 0u;
    *stale = row.voxel.stale ? 1u : 0u;
    *source_flags = row.voxel.source_flags;
    ++x; ++y; ++z; ++rgb; ++valid; ++unknown; ++stale; ++source_flags;
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
    add_path(guide.original_path, color(0.2f, 0.45f, 1.0f, 0.45f), 0.04);
    add_path(guide.risk_path, color(1.0f, 0.52f, 0.04f, 0.65f), 0.06);
    add_path(guide.selected_path,
             guide.risk_selected ? color(0.1f, 0.85f, 0.25f, 1.0f)
                                 : color(0.9f, 0.9f, 0.1f, 0.85f),
             0.10);
    auto label = base_marker(config, stamp, "p4_astar_guides", id++,
                             visualization_msgs::msg::Marker::TEXT_VIEW_FACING);
    label.pose.position = point_msg((guide.segment_start + guide.segment_end) *
                                    0.5 + Eigen::Vector3d(0.0, 0.0, 0.6));
    label.scale.z = 0.22;
    label.color = guide.risk_selected ? color(0.1f, 0.85f, 0.25f, 1.0f)
                                      : color(1.0f, 0.75f, 0.05f, 1.0f);
    label.text = std::string("P4 ") +
                 (guide.risk_selected ? "risk guide" : "original guide") +
                 "\nratio: " + fmt_num(guide.path_length_ratio, 2) +
                 "\nreason: " + guide.reason;
    arr.markers.push_back(label);
  }
  return arr;
}

}  // namespace ego_planner
