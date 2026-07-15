#include <ego_planner/p5_runtime_integrity_gate.h>

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <utility>

namespace ego_planner {
namespace {

constexpr double kTwoPi = 6.28318530717958647692;

std::string jsonNumber(double value) {
  if (!std::isfinite(value)) {
    return "null";
  }
  std::ostringstream oss;
  oss << std::setprecision(12) << value;
  return oss.str();
}

std::string jsonString(const std::string& value) {
  std::ostringstream oss;
  oss << "\"";
  for (const char c : value) {
    switch (c) {
      case '\\':
        oss << "\\\\";
        break;
      case '"':
        oss << "\\\"";
        break;
      case '\n':
        oss << "\\n";
        break;
      case '\r':
        oss << "\\r";
        break;
      case '\t':
        oss << "\\t";
        break;
      default:
        oss << c;
        break;
    }
  }
  oss << "\"";
  return oss.str();
}

std::string jsonStringArray(const std::vector<std::string>& values) {
  std::ostringstream oss;
  oss << "[";
  for (std::size_t i = 0; i < values.size(); ++i) {
    if (i > 0) {
      oss << ",";
    }
    oss << jsonString(values[i]);
  }
  oss << "]";
  return oss.str();
}

std::string jsonTrajectorySamples(
    const std::vector<SafetyVizTrajectorySample>& samples) {
  std::ostringstream oss;
  oss << "[";
  for (std::size_t i = 0; i < samples.size(); ++i) {
    if (i > 0) {
      oss << ",";
    }
    const auto& sample = samples[i];
    oss << "{"
        << "\"tau_s\":" << jsonNumber(sample.tau_s)
        << ",\"query_tau_s\":" << jsonNumber(sample.query_tau_s)
        << ",\"trajectory_start_time_s\":"
        << jsonNumber(sample.trajectory_start_time_s)
        << ",\"trajectory_duration_s\":"
        << jsonNumber(sample.trajectory_duration_s)
        << ",\"trajectory_t_cur_s\":"
        << jsonNumber(sample.trajectory_t_cur_s)
        << ",\"trajectory_t_end_s\":"
        << jsonNumber(sample.trajectory_t_end_s)
        << ",\"trajectory_time_remaining_s\":"
        << jsonNumber(sample.trajectory_time_remaining_s)
        << ",\"sample_dt_s\":" << jsonNumber(sample.sample_dt_s)
        << ",\"horizon_s\":" << jsonNumber(sample.horizon_s)
        << ",\"trajectory_sample_source\":"
        << jsonString(sample.trajectory_sample_source)
        << ",\"x\":" << jsonNumber(sample.position.x())
        << ",\"y\":" << jsonNumber(sample.position.y())
        << ",\"z\":" << jsonNumber(sample.position.z())
        << ",\"hpl\":" << jsonNumber(sample.hpl)
        << ",\"vpl\":" << jsonNumber(sample.vpl)
        << ",\"hal\":" << jsonNumber(sample.hal)
        << ",\"val\":" << jsonNumber(sample.val)
        << ",\"im_min\":" << jsonNumber(sample.im_min)
        << ",\"fixture_match\":"
        << (sample.fixture_match ? "true" : "false")
        << ",\"fixture_expected_hpl\":"
        << jsonNumber(sample.fixture_expected_hpl)
        << ",\"fixture_expected_vpl\":"
        << jsonNumber(sample.fixture_expected_vpl)
        << ",\"fixture_expected_reason\":"
        << jsonString(sample.fixture_expected_reason)
        << ",\"bad\":" << (sample.bad ? "true" : "false")
        << ",\"unknown\":" << (sample.unknown ? "true" : "false")
        << ",\"stale\":" << (sample.stale ? "true" : "false")
        << ",\"reason\":" << jsonString(sample.reason)
        << "}";
  }
  oss << "]";
  return oss.str();
}

P5GateAction maxSeverity(P5GateAction a, P5GateAction b) {
  return static_cast<int>(a) >= static_cast<int>(b) ? a : b;
}

bool isFutureCoverageLimit(const iap::PredictedPLSample& sample) {
  return sample.reason == "time_out_of_horizon";
}

bool isUnknownOnlyFinalGateBlock(const P5GateStatus& status) {
  if (status.action != P5GateAction::REQUEST_REPLAN) {
    return false;
  }
  if (status.reason != P5GateReason::SNAPSHOT_UNAVAILABLE &&
      status.reason != P5GateReason::FUTURE_UNKNOWN &&
      status.reason != P5GateReason::AL_INVALID) {
    return false;
  }
  if (status.bad_count > 0 || status.bad_ratio > 0.0) {
    return false;
  }
  return !std::isfinite(status.future_min_im) ||
         status.future_min_im >= 0.0;
}

bool isTransientCurrentStaleFinalGateBlock(const P5GateStatus& status,
                                           double emergency_duration_s) {
  if (status.action != P5GateAction::REQUEST_REPLAN ||
      status.reason != P5GateReason::CURRENT_STALE) {
    return false;
  }
  if (status.bad_count > 0 || status.bad_ratio > 0.0) {
    return false;
  }
  if (std::isfinite(status.current_im_min) && status.current_im_min < 0.0) {
    return false;
  }
  if (std::isfinite(status.future_min_im) && status.future_min_im < 0.0) {
    return false;
  }
  return status.current_stale_duration_s < emergency_duration_s;
}

bool isLightRiskCurrentLowMarginFinalGateBlock(
    const P5GateStatus& status,
    double current_low_margin_to_emergency_s) {
  if (status.action != P5GateAction::REQUEST_REPLAN ||
      status.reason != P5GateReason::CURRENT_LOW_MARGIN) {
    return false;
  }
  if (status.bad_count > 0 || status.bad_ratio > 0.0) {
    return false;
  }
  if (!std::isfinite(status.future_min_im) || status.future_min_im < 0.0) {
    return false;
  }
  if (status.pred_al_invalid_count > 0 ||
      !std::isfinite(status.pred_hal_min) ||
      !std::isfinite(status.pred_val_min)) {
    return false;
  }
  return status.current_low_margin_duration_s <
         current_low_margin_to_emergency_s;
}

bool pointInsideRegion(const Eigen::Vector3d& p,
                       const Eigen::Vector3d& origin,
                       const Eigen::Vector3d& size) {
  return p.allFinite() && origin.allFinite() && size.allFinite() &&
         p.x() >= origin.x() && p.x() <= origin.x() + size.x() &&
         p.y() >= origin.y() && p.y() <= origin.y() + size.y() &&
         p.z() >= origin.z() && p.z() <= origin.z() + size.z();
}

void appendActiveReason(std::vector<std::string>* reasons,
                        const std::string& reason) {
  if (!reasons || reason.empty() || reason == "ok" ||
      reason == "disabled") {
    return;
  }
  if (std::find(reasons->begin(), reasons->end(), reason) ==
      reasons->end()) {
    reasons->push_back(reason);
  }
}

}  // namespace

PredAlertLimitProvider::PredAlertLimitProvider()
    : PredAlertLimitProvider(Config{}) {}

PredAlertLimitProvider::PredAlertLimitProvider(Config config)
    : config_(std::move(config)) {}

void PredAlertLimitProvider::setConfig(Config config) {
  config_ = std::move(config);
}

void PredAlertLimitProvider::setEnvironment(
    OccupancyQuery occupancy_query,
    MapRegionQuery map_region_query,
    ResolutionQuery resolution_query) {
  occupancy_query_ = std::move(occupancy_query);
  map_region_query_ = std::move(map_region_query);
  resolution_query_ = std::move(resolution_query);
}

PredAlertLimitSample PredAlertLimitProvider::evaluate(
    const Eigen::Vector3d& p_w,
    double /*query_time_s*/,
    double current_hal,
    double current_val) const {
  if (!p_w.allFinite()) {
    PredAlertLimitSample out;
    out.reason = "invalid_position";
    return out;
  }
  switch (config_.mode) {
    case PredAlertLimitMode::CURRENT_MSG_CONSTANT:
      return evaluateCurrent(current_hal, current_val);
    case PredAlertLimitMode::CONFIG_CONSTANT:
      return evaluateConstant();
    case PredAlertLimitMode::OCCUPANCY_CLEARANCE:
      return evaluateOccupancyClearance(p_w);
    case PredAlertLimitMode::VERTICAL_BOUND_ONLY:
      return evaluateVerticalBound(p_w, config_.constant_hal_m);
  }
  PredAlertLimitSample out;
  out.reason = "unknown_mode";
  return out;
}

const char* PredAlertLimitProvider::modeName(PredAlertLimitMode mode) {
  switch (mode) {
    case PredAlertLimitMode::CURRENT_MSG_CONSTANT:
      return "current_msg_constant";
    case PredAlertLimitMode::CONFIG_CONSTANT:
      return "config_constant";
    case PredAlertLimitMode::OCCUPANCY_CLEARANCE:
      return "occupancy_clearance";
    case PredAlertLimitMode::VERTICAL_BOUND_ONLY:
      return "vertical_bound_only";
  }
  return "unknown";
}

PredAlertLimitMode PredAlertLimitProvider::modeFromString(
    const std::string& value) {
  if (value == "config_constant") {
    return PredAlertLimitMode::CONFIG_CONSTANT;
  }
  if (value == "occupancy_clearance") {
    return PredAlertLimitMode::OCCUPANCY_CLEARANCE;
  }
  if (value == "vertical_bound_only") {
    return PredAlertLimitMode::VERTICAL_BOUND_ONLY;
  }
  return PredAlertLimitMode::CURRENT_MSG_CONSTANT;
}

bool PredAlertLimitProvider::finite(double value) {
  return std::isfinite(value);
}

double PredAlertLimitProvider::clampPositive(double value,
                                             double min_value,
                                             double max_value) {
  if (!finite(value) || value < 0.0) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  if (!finite(min_value) || min_value <= 0.0) {
    min_value = 0.1;
  }
  if (!finite(max_value) || max_value < min_value) {
    max_value = min_value;
  }
  return std::clamp(value, min_value, max_value);
}

PredAlertLimitSample PredAlertLimitProvider::evaluateCurrent(
    double current_hal,
    double current_val) const {
  PredAlertLimitSample out;
  out.hal = current_hal;
  out.val = current_val;
  out.valid = finite(out.hal) && finite(out.val) && out.hal > 0.0 &&
              out.val > 0.0;
  out.reason = out.valid ? "ok" : "current_al_invalid";
  return out;
}

PredAlertLimitSample PredAlertLimitProvider::evaluateConstant() const {
  PredAlertLimitSample out;
  out.hal = clampPositive(config_.constant_hal_m, config_.min_hal_m,
                          config_.max_hal_m);
  out.val = clampPositive(config_.constant_val_m, config_.min_val_m,
                          config_.max_val_m);
  out.valid = finite(out.hal) && finite(out.val) && out.hal > 0.0 &&
              out.val > 0.0;
  out.reason = out.valid ? "ok" : "config_al_invalid";
  return out;
}

PredAlertLimitSample PredAlertLimitProvider::evaluateVerticalBound(
    const Eigen::Vector3d& p_w,
    double hal) const {
  Eigen::Vector3d origin;
  Eigen::Vector3d size;
  PredAlertLimitSample out;
  if (!mapRegion(&origin, &size)) {
    out.reason = "map_region_unavailable";
    return out;
  }
  if (!pointInsideRegion(p_w, origin, size)) {
    out.reason = "position_out_of_map";
    return out;
  }
  const double val = verticalLimit(p_w, origin, size);
  out.hal = clampPositive(hal, config_.min_hal_m, config_.max_hal_m);
  out.val = clampPositive(val, config_.min_val_m, config_.max_val_m);
  out.valid = finite(out.hal) && finite(out.val);
  out.reason = out.valid ? "ok" : "position_out_of_map";
  return out;
}

PredAlertLimitSample PredAlertLimitProvider::evaluateOccupancyClearance(
    const Eigen::Vector3d& p_w) const {
  Eigen::Vector3d origin;
  Eigen::Vector3d size;
  PredAlertLimitSample out;
  if (!mapRegion(&origin, &size)) {
    out.reason = "map_region_unavailable";
    return out;
  }
  if (!pointInsideRegion(p_w, origin, size)) {
    out.reason = "position_out_of_map";
    return out;
  }
  const double val = verticalLimit(p_w, origin, size);
  const double clearance = horizontalClearance(p_w);
  out.hal = clampPositive(config_.clearance_scale * clearance,
                          config_.min_hal_m, config_.max_hal_m);
  out.val = clampPositive(val, config_.min_val_m, config_.max_val_m);
  out.valid = finite(out.hal) && finite(out.val);
  if (out.valid) {
    out.reason = "ok";
  } else if (!finite(clearance)) {
    out.reason = "occupancy_query_unavailable";
  } else {
    out.reason = "position_out_of_map";
  }
  return out;
}

bool PredAlertLimitProvider::mapRegion(Eigen::Vector3d* origin,
                                       Eigen::Vector3d* size) const {
  if (!origin || !size || !map_region_query_) {
    return false;
  }
  if (!map_region_query_(origin, size)) {
    return false;
  }
  return origin->allFinite() && size->allFinite() && size->x() > 0.0 &&
         size->y() > 0.0 && size->z() > 0.0;
}

double PredAlertLimitProvider::verticalLimit(
    const Eigen::Vector3d& p_w,
    const Eigen::Vector3d& origin,
    const Eigen::Vector3d& size) const {
  const double z_min = origin.z();
  const double z_max = origin.z() + size.z();
  if (!p_w.allFinite() || p_w.z() < z_min || p_w.z() > z_max) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  const double margin = std::min(p_w.z() - z_min, z_max - p_w.z());
  return config_.vertical_scale * margin;
}

double PredAlertLimitProvider::horizontalClearance(
    const Eigen::Vector3d& p_w) const {
  if (!occupancy_query_) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  double step = config_.clearance_step_m;
  if (resolution_query_) {
    const double resolution = resolution_query_();
    if (finite(resolution) && resolution > 0.0) {
      step = finite(step) && step > 0.0 ? std::min(step, resolution)
                                        : resolution;
    }
  }
  step = finite(step) && step > 0.0 ? std::max(0.05, step) : 0.25;
  const double search_radius =
      finite(config_.clearance_search_radius_m) &&
              config_.clearance_search_radius_m > 0.0
          ? config_.clearance_search_radius_m
          : step;

  if (occupancy_query_(p_w)) {
    return std::max(0.0, -config_.drone_radius_m);
  }
  for (double radius = step; radius <= search_radius + 1.0e-9;
       radius += step) {
    const int samples =
        std::max(8, static_cast<int>(std::ceil(kTwoPi * radius / step)));
    for (int i = 0; i < samples; ++i) {
      const double theta = kTwoPi * static_cast<double>(i) /
                           static_cast<double>(samples);
      Eigen::Vector3d p = p_w;
      p.x() += radius * std::cos(theta);
      p.y() += radius * std::sin(theta);
      if (occupancy_query_(p)) {
        return std::max(0.0, radius - config_.drone_radius_m);
      }
    }
  }
  return config_.max_hal_m;
}

P5RuntimeIntegrityGate::Config P5RuntimeIntegrityGate::declareAndReadConfig(
    const rclcpp::Node::SharedPtr& node) {
  Config config;
  config.enable_runtime_gate =
      node->declare_parameter<bool>("p5.enable_runtime_gate", false);
  config.enable_final_gate =
      node->declare_parameter<bool>("p5.enable_final_gate", false);
  config.horizon_s = node->declare_parameter<double>("p5.horizon_s", 2.0);
  config.sample_dt_s =
      node->declare_parameter<double>("p5.sample_dt_s", 0.2);
  config.current_stale_to_replan_s =
      node->declare_parameter<double>("p5.current_stale_to_replan_s", 0.5);
  config.current_stale_to_emergency_s =
      node->declare_parameter<double>("p5.current_stale_to_emergency_s", 2.0);
  config.current_low_margin_to_emergency_s =
      node->declare_parameter<double>("p5.current_low_margin_to_emergency_s",
                                      2.0);
  config.future_unknown_to_emergency_s =
      node->declare_parameter<double>("p5.future_unknown_to_emergency_s", 2.0);
  config.final_gate_max_consecutive_failures =
      node->declare_parameter<int>("p5.final_gate_max_consecutive_failures", 3);
  config.final_gate_max_failure_duration_s =
      node->declare_parameter<double>("p5.final_gate_max_failure_duration_s",
                                      1.0);
  config.current_replan_margin_m =
      node->declare_parameter<double>("p5.current_replan_margin_m", 0.3);
  config.current_emergency_margin_m =
      node->declare_parameter<double>("p5.current_emergency_margin_m", -0.2);
  config.future_replan_margin_m =
      node->declare_parameter<double>("p5.future_replan_margin_m", 0.3);
  config.future_emergency_margin_m =
      node->declare_parameter<double>("p5.future_emergency_margin_m", -0.5);
  config.max_bad_ratio =
      node->declare_parameter<double>("p5.max_bad_ratio", 0.25);
  config.max_unknown_ratio =
      node->declare_parameter<double>("p5.max_unknown_ratio", 0.30);
  config.bad_tick_to_replan =
      node->declare_parameter<int>("p5.bad_tick_to_replan", 2);
  config.good_tick_to_clear =
      node->declare_parameter<int>("p5.good_tick_to_clear", 2);
  config.pred_alert_limit.mode =
      PredAlertLimitProvider::modeFromString(node->declare_parameter<std::string>(
          "p5.pred_alert_limit_mode", "current_msg_constant"));
  config.pred_alert_limit.constant_hal_m = node->declare_parameter<double>(
      "p5.pred_alert_limit_constant_hal_m", 10.0);
  config.pred_alert_limit.constant_val_m = node->declare_parameter<double>(
      "p5.pred_alert_limit_constant_val_m", 10.0);
  config.pred_alert_limit.min_hal_m = node->declare_parameter<double>(
      "p5.pred_alert_limit_min_hal_m", 0.1);
  config.pred_alert_limit.max_hal_m = node->declare_parameter<double>(
      "p5.pred_alert_limit_max_hal_m", 50.0);
  config.pred_alert_limit.min_val_m = node->declare_parameter<double>(
      "p5.pred_alert_limit_min_val_m", 0.1);
  config.pred_alert_limit.max_val_m = node->declare_parameter<double>(
      "p5.pred_alert_limit_max_val_m", 50.0);
  config.pred_alert_limit.clearance_search_radius_m =
      node->declare_parameter<double>(
          "p5.pred_alert_limit_clearance_search_radius_m", 5.0);
  config.pred_alert_limit.clearance_step_m = node->declare_parameter<double>(
      "p5.pred_alert_limit_clearance_step_m", 0.25);
  config.pred_alert_limit.drone_radius_m = node->declare_parameter<double>(
      "p5.pred_alert_limit_drone_radius_m", 0.35);
  config.pred_alert_limit.clearance_scale = node->declare_parameter<double>(
      "p5.pred_alert_limit_clearance_scale", 1.0);
  config.pred_alert_limit.vertical_scale = node->declare_parameter<double>(
      "p5.pred_alert_limit_vertical_scale", 1.0);
  config.integrity_topic =
      node->declare_parameter<std::string>("p5.integrity_topic",
                                           "/iap/integrity");
  config.status_topic =
      node->declare_parameter<std::string>("p5.status_topic",
                                           "planning/integrity_gate_status");
  config.debug_metrics_enable =
      node->declare_parameter<bool>("p5.debug_metrics_enable", false);
  return config;
}

std::unique_ptr<P5RuntimeIntegrityGate> P5RuntimeIntegrityGate::createIfEnabled(
    const rclcpp::Node::SharedPtr& node) {
  Config config = declareAndReadConfig(node);
  if (!config.enable_runtime_gate && !config.enable_final_gate) {
    return nullptr;
  }
  return std::make_unique<P5RuntimeIntegrityGate>(node, std::move(config));
}

P5RuntimeIntegrityGate::P5RuntimeIntegrityGate(rclcpp::Node::SharedPtr node,
                                               Config config,
                                               bool create_ros_interfaces)
    : node_(std::move(node)),
      config_(std::move(config)),
      pred_alert_limit_provider_(config_.pred_alert_limit) {
  config_.horizon_s = std::max(0.0, config_.horizon_s);
  config_.sample_dt_s = std::max(0.01, config_.sample_dt_s);
  config_.current_stale_to_replan_s =
      std::max(0.0, config_.current_stale_to_replan_s);
  config_.current_stale_to_emergency_s =
      std::max(config_.current_stale_to_replan_s,
               config_.current_stale_to_emergency_s);
  config_.current_low_margin_to_emergency_s =
      std::max(0.0, config_.current_low_margin_to_emergency_s);
  config_.future_unknown_to_emergency_s =
      std::max(0.0, config_.future_unknown_to_emergency_s);
  config_.final_gate_max_consecutive_failures =
      std::max(1, config_.final_gate_max_consecutive_failures);
  config_.final_gate_max_failure_duration_s =
      std::max(0.0, config_.final_gate_max_failure_duration_s);
  config_.max_bad_ratio = std::clamp(config_.max_bad_ratio, 0.0, 1.0);
  config_.max_unknown_ratio = std::clamp(config_.max_unknown_ratio, 0.0, 1.0);
  config_.bad_tick_to_replan = std::max(1, config_.bad_tick_to_replan);
  config_.good_tick_to_clear = std::max(1, config_.good_tick_to_clear);
  config_.pred_alert_limit.min_hal_m =
      std::max(0.001, config_.pred_alert_limit.min_hal_m);
  config_.pred_alert_limit.max_hal_m =
      std::max(config_.pred_alert_limit.min_hal_m,
               config_.pred_alert_limit.max_hal_m);
  config_.pred_alert_limit.min_val_m =
      std::max(0.001, config_.pred_alert_limit.min_val_m);
  config_.pred_alert_limit.max_val_m =
      std::max(config_.pred_alert_limit.min_val_m,
               config_.pred_alert_limit.max_val_m);
  config_.pred_alert_limit.clearance_search_radius_m =
      std::max(0.0, config_.pred_alert_limit.clearance_search_radius_m);
  config_.pred_alert_limit.clearance_step_m =
      std::max(0.01, config_.pred_alert_limit.clearance_step_m);
  config_.pred_alert_limit.drone_radius_m =
      std::max(0.0, config_.pred_alert_limit.drone_radius_m);
  if (!std::isfinite(config_.pred_alert_limit.clearance_scale)) {
    config_.pred_alert_limit.clearance_scale = 1.0;
  }
  if (!std::isfinite(config_.pred_alert_limit.vertical_scale)) {
    config_.pred_alert_limit.vertical_scale = 1.0;
  }
  pred_alert_limit_provider_.setConfig(config_.pred_alert_limit);
  if (create_ros_interfaces) {
    createRosInterfaces();
  }
}

void P5RuntimeIntegrityGate::createRosInterfaces() {
  if (!node_ || (!config_.enable_runtime_gate && !config_.enable_final_gate)) {
    return;
  }
  callback_group_ =
      node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::SubscriptionOptions subscription_options;
  subscription_options.callback_group = callback_group_;
  integrity_sub_ = node_->create_subscription<iap::msg::IntegrityReport>(
      config_.integrity_topic, rclcpp::QoS(20),
      [this](const iap::msg::IntegrityReport::ConstSharedPtr msg) {
        integrityCallback(msg);
      },
      subscription_options);
  if (config_.debug_metrics_enable) {
    status_pub_ =
        node_->create_publisher<std_msgs::msg::String>(config_.status_topic, 10);
  }
  safety_viz_ = std::make_shared<SafetyRvizPublisher>(
      node_, SafetyRvizPublisher::declareAndReadConfig(node_));
}

void P5RuntimeIntegrityGate::integrityCallback(
    const iap::msg::IntegrityReport::ConstSharedPtr msg) {
  if (!msg) {
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  current_ = currentFromMsg(*msg);
}

void P5RuntimeIntegrityGate::setCurrentIntegrityForTest(
    const iap::msg::IntegrityReport& msg) {
  std::lock_guard<std::mutex> lock(mutex_);
  current_ = currentFromMsg(msg);
}

void P5RuntimeIntegrityGate::setPredAlertLimitEnvironment(
    PredAlertLimitProvider::OccupancyQuery occupancy_query,
    PredAlertLimitProvider::MapRegionQuery map_region_query,
    PredAlertLimitProvider::ResolutionQuery resolution_query) {
  pred_alert_limit_provider_.setEnvironment(std::move(occupancy_query),
                                            std::move(map_region_query),
                                            std::move(resolution_query));
}

void P5RuntimeIntegrityGate::resetFinalGateFailureState() {
  final_gate_fail_count_ = 0;
  final_gate_first_failure_s_ = std::numeric_limits<double>::quiet_NaN();
  final_gate_last_reason_ = P5GateReason::OK;
}

P5GateStatus P5RuntimeIntegrityGate::evaluateRuntime(
    LocalTrajData& local_data,
    const std::shared_ptr<const iap::RiskGridSnapshot>& snapshot,
    double now_s,
    double emergency_time_s) {
  if (!config_.enable_runtime_gate) {
    P5GateStatus status;
    status.reason = P5GateReason::DISABLED;
    return status;
  }
  P5GateStatus status = evaluate(local_data, snapshot,
                                 EvalContext{false, now_s, emergency_time_s});
  status = applyDebounce(status, now_s);
  publishStatus(status, "runtime");
  return status;
}

P5GateStatus P5RuntimeIntegrityGate::evaluateFinal(
    LocalTrajData& local_data,
    const std::shared_ptr<const iap::RiskGridSnapshot>& snapshot,
    double now_s,
    double emergency_time_s) {
  if (!config_.enable_final_gate) {
    resetFinalGateFailureState();
    P5GateStatus status;
    status.reason = P5GateReason::DISABLED;
    return status;
  }
  P5GateStatus status = evaluate(local_data, snapshot,
                                 EvalContext{true, now_s, emergency_time_s});
  status = applyFinalGateBudget(status, now_s);
  status.final_candidate_rejected =
      status.action != P5GateAction::OK &&
      status.final_candidate_traj_id >= 0;
  publishStatus(status, "final");
  return status;
}

P5GateStatus P5RuntimeIntegrityGate::evaluate(
    LocalTrajData& local_data,
    const std::shared_ptr<const iap::RiskGridSnapshot>& snapshot,
    const EvalContext& context) {
  P5GateStatus current_status =
      evaluateCurrentGate(context.now_s, !context.final_gate);
  CurrentIntegrity current;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    current = current_;
  }
  P5GateStatus future_status =
      evaluateFutureGate(local_data, snapshot, current, context);
  P5GateStatus merged = merge(current_status, future_status);
  merged.raw_action = merged.action;
  merged.raw_reason = merged.reason;
  return merged;
}

P5GateStatus P5RuntimeIntegrityGate::evaluateCurrentGate(double now_s,
                                                         bool update_state) {
  P5GateStatus status;
  status.reason = P5GateReason::OK;

  CurrentIntegrity current;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    current = current_;
  }

  const bool has_current = current.received && finite(current.stamp_s);
  const double age_s = has_current ? std::max(0.0, now_s - current.stamp_s)
                                   : std::numeric_limits<double>::infinity();
  status.current_integrity_age_s = age_s;

  const bool current_valid =
      has_current && current.valid && finite(current.hpl) && finite(current.vpl) &&
      finite(current.hal) && finite(current.val);
  if (current_valid) {
    status.current_im_h = current.hal - current.hpl;
    status.current_im_v = current.val - current.vpl;
    status.current_im_min = std::min(status.current_im_h, status.current_im_v);
  }

  const bool current_stale =
      current_valid && age_s >= config_.current_stale_to_replan_s;
  const bool stale_or_invalid = !current_valid || current_stale;
  if (stale_or_invalid) {
    if (update_state && !finite(current_problem_started_s_)) {
      current_problem_started_s_ = now_s;
    }
    status.current_stale_duration_s =
        finite(current_problem_started_s_)
            ? std::max(0.0, now_s - current_problem_started_s_)
            : 0.0;
  } else {
    current_problem_started_s_ = std::numeric_limits<double>::quiet_NaN();
    status.current_stale_duration_s = 0.0;
  }

  if (!current_valid) {
    current_low_margin_started_s_ =
        std::numeric_limits<double>::quiet_NaN();
    status.reason = P5GateReason::CURRENT_INVALID;
    status.action = P5GateAction::REQUEST_REPLAN;
    if (status.current_stale_duration_s >=
        config_.current_stale_to_emergency_s) {
      status.action = P5GateAction::REQUEST_EMERGENCY_STOP_CANDIDATE;
    }
    status.raw_action = status.action;
    status.raw_reason = status.reason;
    return status;
  }

  const bool current_low_margin =
      status.current_im_min < config_.current_replan_margin_m;
  if (current_low_margin) {
    if (update_state && !finite(current_low_margin_started_s_)) {
      current_low_margin_started_s_ = now_s;
    }
    status.current_low_margin_duration_s =
        finite(current_low_margin_started_s_)
            ? std::max(0.0, now_s - current_low_margin_started_s_)
            : 0.0;
  } else {
    current_low_margin_started_s_ =
        std::numeric_limits<double>::quiet_NaN();
    status.current_low_margin_duration_s = 0.0;
  }

  if (status.current_im_min < config_.current_emergency_margin_m &&
      status.current_low_margin_duration_s >=
          config_.current_low_margin_to_emergency_s) {
    status.reason = P5GateReason::CURRENT_LOW_MARGIN;
    status.action = P5GateAction::REQUEST_EMERGENCY_STOP_CANDIDATE;
  } else if (status.current_im_min < config_.current_replan_margin_m) {
    status.reason = P5GateReason::CURRENT_LOW_MARGIN;
    status.action = P5GateAction::REQUEST_REPLAN;
  } else if (current_stale &&
             status.current_stale_duration_s >=
                 config_.current_stale_to_emergency_s) {
    status.reason = P5GateReason::CURRENT_STALE;
    status.action = P5GateAction::REQUEST_EMERGENCY_STOP_CANDIDATE;
  } else if (current_stale &&
             status.current_stale_duration_s >=
                 config_.current_stale_to_replan_s) {
    status.reason = P5GateReason::CURRENT_STALE;
    status.action = P5GateAction::REQUEST_REPLAN;
  }

  status.raw_action = status.action;
  status.raw_reason = status.reason;
  return status;
}

P5GateStatus P5RuntimeIntegrityGate::evaluateFutureGate(
    LocalTrajData& local_data,
    const std::shared_ptr<const iap::RiskGridSnapshot>& snapshot,
    const CurrentIntegrity& current,
    const EvalContext& context) {
  P5GateStatus status;
  status.reason = P5GateReason::OK;
  status.pred_al_mode =
      PredAlertLimitProvider::modeName(config_.pred_alert_limit.mode);

  if (!snapshot) {
    status.reason = P5GateReason::SNAPSHOT_UNAVAILABLE;
    status.sample_count = 1;
    status.unknown_count = 1;
    status.unknown_ratio = 1.0;
    status.action = P5GateAction::REQUEST_REPLAN;
    if (!context.final_gate) {
      future_unknown_started_s_ =
          std::numeric_limits<double>::quiet_NaN();
    }
    status.future_unknown_duration_s = 0.0;
    status.raw_action = status.action;
    return status;
  }

  const iap::RiskGridHealth health = snapshot->health();
  status.field_generation_id = health.generation_id;
  status.field_age_s = finite(health.age_s) ? health.age_s
                                            : context.now_s - snapshot->stamp_s();

  const double trajectory_start_time_s = local_data.start_time_.seconds();
  const double duration = std::max(0.0, local_data.duration_);
  if (context.final_gate) {
    status.final_candidate_traj_id = local_data.traj_id_;
    status.final_candidate_start_time_s = trajectory_start_time_s;
    status.final_candidate_duration_s = duration;
  }
  double t_cur = context.now_s - trajectory_start_time_s;
  t_cur = std::clamp(t_cur, 0.0, duration);
  const double t_end = std::min(duration, t_cur + config_.horizon_s);
  const double dt = std::max(0.01, config_.sample_dt_s);
  const double time_remaining = std::max(0.0, duration - t_cur);
  const std::string sample_source =
      context.final_gate ? "final_candidate" : "runtime_committed";

  auto fill_timing = [&](SafetyVizTrajectorySample* sample) {
    if (!sample) {
      return;
    }
    sample->trajectory_start_time_s = trajectory_start_time_s;
    sample->trajectory_duration_s = duration;
    sample->trajectory_t_cur_s = t_cur;
    sample->trajectory_t_end_s = t_end;
    sample->trajectory_time_remaining_s = time_remaining;
    sample->sample_dt_s = dt;
    sample->horizon_s = config_.horizon_s;
    sample->trajectory_sample_source = sample_source;
  };

  bool emitted_trajectory_timing_failure = false;
  if (duration <= 1.0e-9 || time_remaining <= 1.0e-9 ||
      t_end <= t_cur + 1.0e-9) {
    SafetyVizTrajectorySample viz_sample;
    fill_timing(&viz_sample);
    viz_sample.tau_s = 0.0;
    viz_sample.query_tau_s = 0.0;
    viz_sample.position = local_data.position_traj_.evaluateDeBoorT(t_cur);
    viz_sample.unknown = true;
    viz_sample.reason =
        duration <= 1.0e-9 ? "trajectory_zero_duration"
                           : "trajectory_expired";
    status.sample_count = 1;
    status.unknown_count = 1;
    status.unknown_ratio = 1.0;
    status.viz_samples.push_back(viz_sample);
    emitted_trajectory_timing_failure = true;
  }

  for (double t = t_cur;
       !emitted_trajectory_timing_failure && t <= t_end + 1.0e-9;
       t += dt) {
    const double tau = std::max(0.0, t - t_cur);
    const Eigen::Vector3d p = local_data.position_traj_.evaluateDeBoorT(t);
    SafetyVizTrajectorySample viz_sample;
    fill_timing(&viz_sample);
    viz_sample.position = p;
    viz_sample.tau_s = tau;
    iap::PredictedPLSample pl;
    const bool pl_ok =
        snapshot->queryPredictedPL(p, context.now_s + tau, &pl, tau,
                                   context.final_gate);
    const PredAlertLimitSample al = pred_alert_limit_provider_.evaluate(
        p, context.now_s + tau, current.hal, current.val);
    if (!pl_ok && isFutureCoverageLimit(pl)) {
      continue;
    }
    status.sample_count++;
    viz_sample.query_tau_s = pl.query_tau_s;
    viz_sample.hpl = pl.hpl_pred;
    viz_sample.vpl = pl.vpl_pred;
    viz_sample.hal = al.hal;
    viz_sample.val = al.val;
    viz_sample.fixture_match = pl.fixture_match;
    viz_sample.fixture_expected_hpl = pl.fixture_expected_hpl;
    viz_sample.fixture_expected_vpl = pl.fixture_expected_vpl;
    viz_sample.fixture_expected_reason = pl.fixture_expected_reason;
    if (!al.valid) {
      status.pred_al_invalid_count++;
      status.pred_al_last_reason = al.reason;
    } else {
      if (!finite(status.pred_hal_min) || al.hal < status.pred_hal_min) {
        status.pred_hal_min = al.hal;
      }
      if (!finite(status.pred_val_min) || al.val < status.pred_val_min) {
        status.pred_val_min = al.val;
      }
    }
    if (!al.valid || !pl_ok || !pl.available || !pl.valid || pl.stale ||
        !finite(pl.hpl_pred) || !finite(pl.vpl_pred)) {
      status.unknown_count++;
      viz_sample.unknown = true;
      viz_sample.stale = pl.stale;
      viz_sample.reason = !al.valid ? al.reason : pl.reason;
      status.viz_samples.push_back(viz_sample);
      continue;
    }

    const double im = std::min(al.hal - pl.hpl_pred, al.val - pl.vpl_pred);
    viz_sample.im_min = im;
    viz_sample.bad = im < config_.future_replan_margin_m;
    viz_sample.good = !viz_sample.bad;
    const std::string pl_reason =
        pl.reason.empty() ? std::string("ok") : pl.reason;
    viz_sample.reason =
        viz_sample.bad && pl_reason != "ok"
            ? "future_low_margin:" + pl_reason
            : (viz_sample.bad ? "future_low_margin" : pl_reason);
    status.viz_samples.push_back(viz_sample);
    if (!finite(status.future_min_im) || im < status.future_min_im) {
      status.future_min_im = im;
    }
    if (im < config_.future_replan_margin_m) {
      status.bad_count++;
      if (!finite(status.first_bad_tau)) {
        status.first_bad_tau = tau;
      }
    }
  }

  if (status.sample_count <= 0) {
    status.sample_count = 1;
    status.unknown_count = 1;
  }
  status.bad_ratio =
      static_cast<double>(status.bad_count) / status.sample_count;
  status.unknown_ratio =
      static_cast<double>(status.unknown_count) / status.sample_count;

  const bool unknown_high = status.unknown_ratio >= config_.max_unknown_ratio;
  const bool debounce_eligible_unknown =
      unknown_high && !emitted_trajectory_timing_failure;
  if (debounce_eligible_unknown) {
    if (!context.final_gate && !finite(future_unknown_started_s_)) {
      future_unknown_started_s_ = context.now_s;
    }
    status.future_unknown_duration_s =
        finite(future_unknown_started_s_)
            ? std::max(0.0, context.now_s - future_unknown_started_s_)
            : 0.0;
  } else {
    future_unknown_started_s_ = std::numeric_limits<double>::quiet_NaN();
    status.future_unknown_duration_s = 0.0;
  }

  if (finite(status.first_bad_tau) &&
      status.first_bad_tau <= context.emergency_time_s) {
    status.reason = P5GateReason::FUTURE_BAD;
    status.action = P5GateAction::REQUEST_EMERGENCY_STOP_CANDIDATE;
  } else if (finite(status.future_min_im) &&
             status.future_min_im < config_.future_emergency_margin_m) {
    status.reason = P5GateReason::FUTURE_BAD;
    status.action = P5GateAction::REQUEST_EMERGENCY_STOP_CANDIDATE;
  } else if (status.bad_ratio >= config_.max_bad_ratio) {
    status.reason = P5GateReason::FUTURE_BAD;
    status.action = P5GateAction::REQUEST_REPLAN;
  } else if (unknown_high) {
    status.reason = status.pred_al_invalid_count > 0 ? P5GateReason::AL_INVALID
                                                     : P5GateReason::FUTURE_UNKNOWN;
    status.action = debounce_eligible_unknown &&
                            status.future_unknown_duration_s >=
                                config_.future_unknown_to_emergency_s
                        ? P5GateAction::REQUEST_EMERGENCY_STOP_CANDIDATE
                        : P5GateAction::REQUEST_REPLAN;
  }

  status.raw_action = status.action;
  status.raw_reason = status.reason;
  return status;
}

P5GateStatus P5RuntimeIntegrityGate::merge(const P5GateStatus& a,
                                           const P5GateStatus& b) const {
  P5GateStatus out = a;
  out.current_im_h = a.current_im_h;
  out.current_im_v = a.current_im_v;
  out.current_im_min = a.current_im_min;
  out.future_min_im = b.future_min_im;
  out.first_bad_tau = b.first_bad_tau;
  out.bad_ratio = b.bad_ratio;
  out.unknown_ratio = b.unknown_ratio;
  out.current_integrity_age_s = a.current_integrity_age_s;
  out.field_generation_id = b.field_generation_id;
  out.field_age_s = b.field_age_s;
  out.current_stale_duration_s = a.current_stale_duration_s;
  out.current_low_margin_duration_s = a.current_low_margin_duration_s;
  out.future_unknown_duration_s = b.future_unknown_duration_s;
  out.final_candidate_traj_id = b.final_candidate_traj_id;
  out.final_candidate_start_time_s = b.final_candidate_start_time_s;
  out.final_candidate_duration_s = b.final_candidate_duration_s;
  out.final_candidate_rejected = b.final_candidate_rejected;
  out.pred_al_mode = b.pred_al_mode;
  out.pred_hal_min = b.pred_hal_min;
  out.pred_val_min = b.pred_val_min;
  out.pred_al_invalid_count = b.pred_al_invalid_count;
  out.pred_al_last_reason = b.pred_al_last_reason;
  out.sample_count = b.sample_count;
  out.bad_count = b.bad_count;
  out.unknown_count = b.unknown_count;
  out.viz_samples = b.viz_samples;
  out.current_reason =
      a.action != P5GateAction::OK ? reasonName(a.reason) : "";
  out.future_reason =
      b.action != P5GateAction::OK ? reasonName(b.reason) : "";
  out.active_reasons.clear();
  appendActiveReason(&out.active_reasons, out.current_reason);
  appendActiveReason(&out.active_reasons, out.future_reason);

  out.action = maxSeverity(a.action, b.action);
  if (out.action == b.action && b.action != P5GateAction::OK) {
    out.reason = b.reason;
  } else if (a.action != P5GateAction::OK) {
    out.reason = a.reason;
  } else if (b.action != P5GateAction::OK) {
    out.reason = b.reason;
  } else {
    out.reason = P5GateReason::OK;
  }
  out.raw_action = out.action;
  out.raw_reason = out.reason;
  return out;
}

P5GateStatus P5RuntimeIntegrityGate::applyDebounce(const P5GateStatus& raw,
                                                   double now_s) {
  P5GateStatus status = raw;
  status.raw_action = raw.action;
  status.raw_reason = raw.reason;

  if (raw.action == P5GateAction::REQUEST_EMERGENCY_STOP_CANDIDATE) {
    bad_ticks_ = 0;
    good_ticks_ = 0;
    return status;
  }

  if (raw.action == P5GateAction::REQUEST_REPLAN) {
    bad_ticks_++;
    good_ticks_ = 0;
    if (bad_ticks_ < config_.bad_tick_to_replan) {
      status.action = P5GateAction::OK;
    }
    return status;
  }

  good_ticks_++;
  const bool current_stale_candidate =
      finite(raw.current_integrity_age_s) &&
      raw.current_integrity_age_s >= config_.current_stale_to_replan_s;
  if (good_ticks_ >= config_.good_tick_to_clear) {
    bad_ticks_ = 0;
    future_unknown_started_s_ = std::numeric_limits<double>::quiet_NaN();
    if (!current_stale_candidate) {
      current_problem_started_s_ = std::numeric_limits<double>::quiet_NaN();
    }
    current_low_margin_started_s_ =
        std::numeric_limits<double>::quiet_NaN();
  }
  status.current_stale_duration_s =
      finite(current_problem_started_s_)
          ? std::max(0.0, now_s - current_problem_started_s_)
          : 0.0;
  status.future_unknown_duration_s =
      finite(future_unknown_started_s_)
          ? std::max(0.0, now_s - future_unknown_started_s_)
          : 0.0;
  return status;
}

P5GateStatus P5RuntimeIntegrityGate::applyFinalGateBudget(
    const P5GateStatus& raw,
    double now_s) {
  P5GateStatus status = raw;
  status.raw_action = raw.action;
  status.raw_reason = raw.reason;

  if (raw.action == P5GateAction::OK) {
    resetFinalGateFailureState();
    status.final_gate_fail_count = 0;
    status.final_gate_fail_duration_s = 0.0;
    status.final_gate_last_reason.clear();
    return status;
  }

  if (isUnknownOnlyFinalGateBlock(raw)) {
    resetFinalGateFailureState();
    status.final_gate_fail_count = 0;
    status.final_gate_fail_duration_s = 0.0;
    status.final_gate_last_reason.clear();
    return status;
  }

  if (isTransientCurrentStaleFinalGateBlock(
          raw, config_.current_stale_to_emergency_s)) {
    resetFinalGateFailureState();
    status.final_gate_fail_count = 0;
    status.final_gate_fail_duration_s = 0.0;
    status.final_gate_last_reason.clear();
    return status;
  }

  if (isLightRiskCurrentLowMarginFinalGateBlock(
          raw, config_.current_low_margin_to_emergency_s)) {
    resetFinalGateFailureState();
    status.final_gate_fail_count = 0;
    status.final_gate_fail_duration_s = 0.0;
    status.final_gate_last_reason.clear();
    return status;
  }

  if (!finite(final_gate_first_failure_s_)) {
    final_gate_first_failure_s_ = now_s;
  }
  final_gate_fail_count_++;
  final_gate_last_reason_ = raw.reason;

  status.final_gate_fail_count = final_gate_fail_count_;
  status.final_gate_fail_duration_s =
      finite(final_gate_first_failure_s_)
          ? std::max(0.0, now_s - final_gate_first_failure_s_)
          : 0.0;
  status.final_gate_last_reason = reasonName(final_gate_last_reason_);

  const bool count_exceeded =
      final_gate_fail_count_ >= config_.final_gate_max_consecutive_failures;
  const bool duration_exceeded =
      status.final_gate_fail_duration_s >=
      config_.final_gate_max_failure_duration_s;
  if (raw.action != P5GateAction::REQUEST_EMERGENCY_STOP_CANDIDATE &&
      (count_exceeded || duration_exceeded)) {
    status.action = P5GateAction::REQUEST_EMERGENCY_STOP_CANDIDATE;
    status.reason = P5GateReason::FINAL_GATE_FAILED;
  }
  return status;
}

void P5RuntimeIntegrityGate::publishStatus(const P5GateStatus& status,
                                           const std::string& phase) {
  if (safety_viz_) {
    SafetyVizGateStatus viz_status;
    viz_status.phase = phase;
    viz_status.action = actionName(status.action);
    viz_status.reason = reasonName(status.reason);
    viz_status.current_im_h = status.current_im_h;
    viz_status.current_im_v = status.current_im_v;
    viz_status.current_im_min = status.current_im_min;
    viz_status.future_min_im = status.future_min_im;
    viz_status.first_bad_tau = status.first_bad_tau;
    viz_status.bad_ratio = status.bad_ratio;
    viz_status.unknown_ratio = status.unknown_ratio;
    viz_status.sample_count = status.sample_count;
    viz_status.bad_count = status.bad_count;
    viz_status.unknown_count = status.unknown_count;
    viz_status.samples = status.viz_samples;
    const double now_s = node_ ? node_->now().seconds()
                               : std::numeric_limits<double>::quiet_NaN();
    safety_viz_->publishP5GateStatus(viz_status, now_s);
  }
  if (!status_pub_) {
    return;
  }
  std_msgs::msg::String msg;
  msg.data = toJson(status, phase);
  status_pub_->publish(msg);
}

std::string P5RuntimeIntegrityGate::toJson(
    const P5GateStatus& status,
    const std::string& phase) const {
  std::ostringstream oss;
  oss << "{\"phase\":" << jsonString(phase)
      << ",\"action\":" << jsonString(actionName(status.action))
      << ",\"raw_action\":" << jsonString(actionName(status.raw_action))
      << ",\"reason\":" << jsonString(reasonName(status.reason))
      << ",\"raw_reason\":" << jsonString(reasonName(status.raw_reason))
      << ",\"current_reason\":" << jsonString(status.current_reason)
      << ",\"future_reason\":" << jsonString(status.future_reason)
      << ",\"active_reasons\":" << jsonStringArray(status.active_reasons)
      << ",\"current_im_h\":" << jsonNumber(status.current_im_h)
      << ",\"current_im_v\":" << jsonNumber(status.current_im_v)
      << ",\"current_im_min\":" << jsonNumber(status.current_im_min)
      << ",\"future_min_im\":" << jsonNumber(status.future_min_im)
      << ",\"first_bad_tau\":" << jsonNumber(status.first_bad_tau)
      << ",\"bad_ratio\":" << jsonNumber(status.bad_ratio)
      << ",\"unknown_ratio\":" << jsonNumber(status.unknown_ratio)
      << ",\"current_integrity_age_s\":"
      << jsonNumber(status.current_integrity_age_s)
      << ",\"field_generation_id\":" << status.field_generation_id
      << ",\"field_age_s\":" << jsonNumber(status.field_age_s)
      << ",\"current_stale_duration_s\":"
      << jsonNumber(status.current_stale_duration_s)
      << ",\"current_low_margin_duration_s\":"
      << jsonNumber(status.current_low_margin_duration_s)
      << ",\"future_unknown_duration_s\":"
      << jsonNumber(status.future_unknown_duration_s)
      << ",\"final_gate_fail_count\":" << status.final_gate_fail_count
      << ",\"final_gate_fail_duration_s\":"
      << jsonNumber(status.final_gate_fail_duration_s)
      << ",\"final_gate_last_reason\":"
      << jsonString(status.final_gate_last_reason)
      << ",\"final_candidate_traj_id\":"
      << status.final_candidate_traj_id
      << ",\"final_candidate_start_time_s\":"
      << jsonNumber(status.final_candidate_start_time_s)
      << ",\"final_candidate_duration_s\":"
      << jsonNumber(status.final_candidate_duration_s)
      << ",\"final_candidate_rejected\":"
      << (status.final_candidate_rejected ? "true" : "false")
      << ",\"pred_al_mode\":" << jsonString(status.pred_al_mode)
      << ",\"pred_hal_min\":" << jsonNumber(status.pred_hal_min)
      << ",\"pred_val_min\":" << jsonNumber(status.pred_val_min)
      << ",\"pred_al_invalid_count\":" << status.pred_al_invalid_count
      << ",\"pred_al_last_reason\":"
      << jsonString(status.pred_al_last_reason)
      << ",\"sample_count\":" << status.sample_count
      << ",\"bad_count\":" << status.bad_count
      << ",\"unknown_count\":" << status.unknown_count
      << ",\"samples\":" << jsonTrajectorySamples(status.viz_samples) << "}";
  return oss.str();
}

double P5RuntimeIntegrityGate::stampToSec(
    const builtin_interfaces::msg::Time& stamp) {
  return static_cast<double>(stamp.sec) +
         1.0e-9 * static_cast<double>(stamp.nanosec);
}

bool P5RuntimeIntegrityGate::finite(double value) {
  return std::isfinite(value);
}

P5RuntimeIntegrityGate::CurrentIntegrity
P5RuntimeIntegrityGate::currentFromMsg(const iap::msg::IntegrityReport& msg) {
  CurrentIntegrity current;
  current.received = true;
  current.stamp_s = stampToSec(msg.header.stamp);
  current.hpl = msg.hpl;
  current.vpl = msg.vpl;
  current.hal = msg.hal;
  current.val = msg.val;
  current.im = msg.im;
  current.valid = finite(current.stamp_s) && finite(current.hpl) &&
                  finite(current.vpl) && finite(current.hal) &&
                  finite(current.val) && finite(current.im) &&
                  !msg.hal_invalid && !msg.val_invalid && !msg.im_invalid;
  return current;
}

const char* P5RuntimeIntegrityGate::actionName(P5GateAction action) {
  switch (action) {
    case P5GateAction::OK:
      return "OK";
    case P5GateAction::REQUEST_REPLAN:
      return "REQUEST_REPLAN";
    case P5GateAction::REQUEST_EMERGENCY_STOP_CANDIDATE:
      return "REQUEST_EMERGENCY_STOP_CANDIDATE";
  }
  return "UNKNOWN";
}

const char* P5RuntimeIntegrityGate::reasonName(P5GateReason reason) {
  switch (reason) {
    case P5GateReason::DISABLED:
      return "disabled";
    case P5GateReason::OK:
      return "ok";
    case P5GateReason::CURRENT_INVALID:
      return "current_invalid";
    case P5GateReason::CURRENT_STALE:
      return "current_stale";
    case P5GateReason::CURRENT_LOW_MARGIN:
      return "current_low_margin";
    case P5GateReason::FUTURE_BAD:
      return "future_bad";
    case P5GateReason::FUTURE_UNKNOWN:
      return "future_unknown";
    case P5GateReason::AL_INVALID:
      return "al_invalid";
    case P5GateReason::FINAL_GATE_FAILED:
      return "final_gate_failed";
    case P5GateReason::SNAPSHOT_UNAVAILABLE:
      return "snapshot_unavailable";
  }
  return "unknown";
}

}  // namespace ego_planner
