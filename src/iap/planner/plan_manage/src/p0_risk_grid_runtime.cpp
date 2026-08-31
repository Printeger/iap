#include <ego_planner/p0_risk_grid_runtime.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <ctime>
#include <exception>
#include <future>
#include <iomanip>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>
#include <utility>

#include <gnss_comm/gnss_constant.hpp>
#include <gnss_comm/gnss_ros.hpp>
#include <gnss_comm/gnss_utility.hpp>
#include <iap/planner/predictor_risk_conversion.hpp>
#include <iap/predictor/predictor_module.hpp>
#include <Eigen/Eigenvalues>
#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace ego_planner {
namespace {

constexpr double kLightSpeed = 2.99792458e8;

template <typename T>
bool sameSharedOwner(const std::shared_ptr<const T>& lhs,
                     const std::shared_ptr<const T>& rhs) {
  return !lhs.owner_before(rhs) && !rhs.owner_before(lhs);
}

enum class P0SemanticFailure {
  NONE = 0,
  OCCUPANCY_SNAPSHOT_UNAVAILABLE,
  OCCUPANCY_LOS_ADAPTER_INVALID,
  OCCUPANCY_FRAME_MISMATCH,
  OCCUPANCY_STALE,
  OCCUPANCY_GENERATION_CHANGED,
  PRIOR_GENERATION_CHANGED,
  INVALID_COVARIANCE_GROWTH_PARAMETER,
  MISSING_COVARIANCE_GROWTH_PRIOR,
  STALE_COVARIANCE_GROWTH_PRIOR,
  INVALID_COVARIANCE_GROWTH_PRIOR,
};

const char* semanticFailureReason(const P0SemanticFailure failure) {
  switch (failure) {
    case P0SemanticFailure::NONE:
      return "none";
    case P0SemanticFailure::OCCUPANCY_SNAPSHOT_UNAVAILABLE:
      return "occupancy_snapshot_unavailable";
    case P0SemanticFailure::OCCUPANCY_LOS_ADAPTER_INVALID:
      return "occupancy_los_adapter_invalid";
    case P0SemanticFailure::OCCUPANCY_FRAME_MISMATCH:
      return "occupancy_frame_mismatch";
    case P0SemanticFailure::OCCUPANCY_STALE:
      return "occupancy_stale";
    case P0SemanticFailure::OCCUPANCY_GENERATION_CHANGED:
      return "occupancy_generation_changed";
    case P0SemanticFailure::PRIOR_GENERATION_CHANGED:
      return "prior_generation_changed";
    case P0SemanticFailure::INVALID_COVARIANCE_GROWTH_PARAMETER:
      return "invalid_covariance_growth_parameter";
    case P0SemanticFailure::MISSING_COVARIANCE_GROWTH_PRIOR:
      return "missing_covariance_growth_prior";
    case P0SemanticFailure::STALE_COVARIANCE_GROWTH_PRIOR:
      return "stale_covariance_growth_prior";
    case P0SemanticFailure::INVALID_COVARIANCE_GROWTH_PRIOR:
      return "invalid_covariance_growth_prior";
  }
  return "invalid_semantic_failure";
}

bool finiteSpd(const Eigen::Matrix3d& matrix) {
  if (!matrix.allFinite()) {
    return false;
  }
  const double scale = std::max(1.0, matrix.cwiseAbs().maxCoeff());
  if ((matrix - matrix.transpose()).cwiseAbs().maxCoeff() >
      1.0e-12 * scale) {
    return false;
  }
  const Eigen::Matrix3d symmetric = 0.5 * (matrix + matrix.transpose());
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen(
      symmetric, Eigen::EigenvaluesOnly);
  return eigen.info() == Eigen::Success &&
         eigen.eigenvalues().allFinite() &&
         eigen.eigenvalues().minCoeff() > 0.0;
}

double stampToSec(const builtin_interfaces::msg::Time& stamp) {
  return static_cast<double>(stamp.sec) +
         1.0e-9 * static_cast<double>(stamp.nanosec);
}

bool finite(double value) {
  return std::isfinite(value);
}

void advanceNonzeroGeneration(uint64_t* generation) {
  if (!generation) return;
  ++*generation;
  if (*generation == 0u) ++*generation;
}

double steadyNowSeconds() {
  static const auto epoch = std::chrono::steady_clock::now();
  return std::chrono::duration<double>(std::chrono::steady_clock::now() - epoch)
      .count();
}

std::string jsonNumber(double value) {
  if (!std::isfinite(value)) {
    return "null";
  }
  std::ostringstream oss;
  oss << std::setprecision(17) << value;
  return oss.str();
}

std::string jsonString(const std::string& value) {
  std::ostringstream oss;
  oss << '"';
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
  oss << '"';
  return oss.str();
}

std::string jsonBool(bool value) {
  return value ? "true" : "false";
}

bool hasPointField(const sensor_msgs::msg::PointCloud2& msg,
                   const std::string& name) {
  return std::any_of(msg.fields.begin(), msg.fields.end(),
                     [&](const auto& field) {
                       return field.name == name;
                     });
}

struct PredictorPositionCacheKey {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;

  bool operator==(const PredictorPositionCacheKey& other) const {
    return x == other.x && y == other.y && z == other.z;
  }
};

struct PredictorPositionCacheHash {
  std::size_t operator()(const PredictorPositionCacheKey& key) const {
    std::size_t seed = std::hash<double>{}(key.x);
    seed ^= std::hash<double>{}(key.y) + 0x9e3779b9u + (seed << 6) +
            (seed >> 2);
    seed ^= std::hash<double>{}(key.z) + 0x9e3779b9u + (seed << 6) +
            (seed >> 2);
    return seed;
  }
};

PredictorPositionCacheKey makePredictorPositionCacheKey(
    const Eigen::Vector3d& position) {
  return PredictorPositionCacheKey{position.x(), position.y(), position.z()};
}

iap::PredictorSourceMode parsePredictorSourceMode(
    const std::string& value) {
  if (value == "fusion") {
    return iap::PredictorSourceMode::Fusion;
  }
  if (value == "gnss_only") {
    return iap::PredictorSourceMode::GnssOnly;
  }
  if (value == "lidar_only") {
    return iap::PredictorSourceMode::LidarOnly;
  }
  throw std::invalid_argument(
      "invalid p0.predictor.source_mode '" + value +
      "'; expected one of: fusion, gnss_only, lidar_only");
}

iap::PredictorGnssEpochPolicy parsePredictorGnssEpochPolicy(
    const std::string& value) {
  if (value == "auto") {
    return iap::PredictorGnssEpochPolicy::Auto;
  }
  if (value == "required") {
    return iap::PredictorGnssEpochPolicy::Required;
  }
  if (value == "optional") {
    return iap::PredictorGnssEpochPolicy::Optional;
  }
  if (value == "disabled") {
    return iap::PredictorGnssEpochPolicy::Disabled;
  }
  throw std::invalid_argument(
      "invalid p0.predictor.gnss_epoch_policy '" + value +
      "'; expected one of: auto, required, optional, disabled");
}

class PredictorModuleRiskProvider final : public iap::RiskPredictionProvider {
 public:
  PredictorModuleRiskProvider(
      iap::RollingSpatialAdvisoryWindow* rolling_window,
      iap::RollingSpatialWindowGeometry geometry,
      std::shared_ptr<const iap::LocalOccupancyGrid> occupancy_owner,
      std::shared_ptr<const std::vector<Eigen::Vector3d>> lidar_map_points,
      std::shared_ptr<const std::vector<iap::LidarFimPrimitive>>
          lidar_fim_primitives,
      iap::RollingSpatialRetentionPolicy retention_policy,
      iap::RollingSpatialSourceProvenance source_provenance,
      iap::PredictorModule module,
      iap::IntegritySnapshot snapshot,
      int worker_count, double hal_m, double val_m)
      : occupancy_owner_(std::move(occupancy_owner)),
        rolling_window_(rolling_window), snapshot_(std::move(snapshot)),
        worker_count_(std::max(1, worker_count)),
        hal_m_(hal_m), val_m_(val_m) {
    iap::RollingSpatialRefreshInput input;
    input.geometry = std::move(geometry);
    input.module = std::move(module);
    input.snapshot = snapshot_;
    input.policy = retention_policy;
    input.provenance = source_provenance;
    input.occupancy_owner = occupancy_owner_;
    input.lidar_map_points_owner = std::move(lidar_map_points);
    input.lidar_fim_primitives_owner = std::move(lidar_fim_primitives);
    ready_ = rolling_window_ && rolling_window_->beginRefresh(
        std::move(input), &begin_failure_reason_);
    if (rolling_window_ && !ready_) {
      begin_diagnostics_ = rolling_window_->diagnostics();
    }
  }

  ~PredictorModuleRiskProvider() override {
    if (rolling_window_ && ready_) {
      rolling_window_->abortRefresh();
    }
  }

  bool batchQuery(const std::vector<iap::RiskPredictionQuery>& queries,
                  std::vector<iap::RiskPredictionResult>* results) override {
    if (results == nullptr || !ready_) {
      return false;
    }
    results->assign(queries.size(), iap::RiskPredictionResult{});
    if (queries.empty()) {
      return true;
    }
    std::unordered_map<PredictorPositionCacheKey, std::size_t,
                       PredictorPositionCacheHash> group_by_position;
    std::vector<std::vector<std::size_t>> groups;
    group_by_position.reserve(queries.size());
    for (std::size_t index = 0; index < queries.size(); ++index) {
      const auto key = makePredictorPositionCacheKey(queries[index].position_w);
      const auto inserted = group_by_position.emplace(key, groups.size());
      if (inserted.second) groups.emplace_back();
      groups[inserted.first->second].push_back(index);
    }
    const int worker_count = std::min<int>(worker_count_, groups.size());
    struct WorkerOutcome {
      iap::PredictorBatchDiagnostics diagnostics;
      bool growth_valid = true;
    };
    std::vector<std::future<WorkerOutcome>> workers;
    workers.reserve(static_cast<std::size_t>(worker_count));
    for (int worker_id = 0; worker_id < worker_count; ++worker_id) {
      workers.push_back(std::async(std::launch::async,
          [this, &queries, &groups, results, worker_id, worker_count]() {
            WorkerOutcome outcome;
            for (std::size_t group_index = static_cast<std::size_t>(worker_id);
                 group_index < groups.size();
                 group_index += static_cast<std::size_t>(worker_count)) {
              std::vector<iap::PredictorQueryInput> inputs;
              inputs.reserve(groups[group_index].size());
              for (const std::size_t index : groups[group_index]) {
                const auto& query = queries[index];
                inputs.emplace_back(query.position_w, snapshot_,
                    query.query_time_s, query.horizon_s, "map", snapshot_.stamp);
              }
              iap::PredictorBatchDiagnostics diagnostics;
              const auto predictions =
                  rolling_window_->queryPositionHorizons(inputs, &diagnostics);
              if (predictions.size() != inputs.size()) {
                outcome.growth_valid = false;
                return outcome;
              }
              for (std::size_t local = 0; local < predictions.size(); ++local) {
                if (inputs[local].horizon_s > 0.0 &&
                    predictions[local].covariance_growth_status !=
                        iap::CovarianceGrowthStatus::APPLIED) {
                  outcome.growth_valid = false;
                }
                (*results)[groups[group_index][local]] =
                    iap::makeRiskPredictionResult(
                        predictions[local], hal_m_, val_m_);
              }
              outcome.diagnostics.query_count += diagnostics.query_count;
              outcome.diagnostics.unique_positions += diagnostics.unique_positions;
              outcome.diagnostics.lidar_evaluations += diagnostics.lidar_evaluations;
              outcome.diagnostics.lidar_cache_hits += diagnostics.lidar_cache_hits;
              outcome.diagnostics.spatial_advisory_recompute_count +=
                  diagnostics.spatial_advisory_recompute_count;
              outcome.diagnostics.spatial_advisory_reuse_count +=
                  diagnostics.spatial_advisory_reuse_count;
              outcome.diagnostics.gnss_advisory_invocations +=
                  diagnostics.gnss_advisory_invocations;
              outcome.diagnostics.lidar_advisory_invocations +=
                  diagnostics.lidar_advisory_invocations;
              outcome.diagnostics.fusion_advisory_invocations +=
                  diagnostics.fusion_advisory_invocations;
            }
            return outcome;
          }));
    }
    last_diagnostics_ = {};
    bool growth_valid = true;
    for (auto& worker : workers) {
      const auto outcome = worker.get();
      const auto& diagnostics = outcome.diagnostics;
      growth_valid = growth_valid && outcome.growth_valid;
      last_diagnostics_.query_count += diagnostics.query_count;
      last_diagnostics_.unique_positions += diagnostics.unique_positions;
      last_diagnostics_.lidar_evaluations += diagnostics.lidar_evaluations;
      last_diagnostics_.lidar_cache_hits += diagnostics.lidar_cache_hits;
      last_diagnostics_.spatial_advisory_recompute_count +=
          diagnostics.spatial_advisory_recompute_count;
      last_diagnostics_.spatial_advisory_reuse_count +=
          diagnostics.spatial_advisory_reuse_count;
      last_diagnostics_.gnss_advisory_invocations +=
          diagnostics.gnss_advisory_invocations;
      last_diagnostics_.lidar_advisory_invocations +=
          diagnostics.lidar_advisory_invocations;
      last_diagnostics_.fusion_advisory_invocations +=
          diagnostics.fusion_advisory_invocations;
    }
    return growth_valid;
  }

  const iap::PredictorBatchDiagnostics& diagnostics() const {
    return last_diagnostics_;
  }

  iap::RollingSpatialRefreshDiagnostics rollingDiagnostics() const {
    return ready_ && rolling_window_ ? rolling_window_->diagnostics()
                                    : begin_diagnostics_;
  }

  bool ready() const { return ready_; }
  const std::string& beginFailureReason() const {
    return begin_failure_reason_;
  }

  void finish(const bool success) {
    if (!rolling_window_ || !ready_) return;
    if (success) {
      rolling_window_->commitRefresh();
    } else {
      rolling_window_->abortRefresh();
    }
    ready_ = false;
  }

 private:
  std::shared_ptr<const iap::LocalOccupancyGrid> occupancy_owner_;
  iap::RollingSpatialAdvisoryWindow* rolling_window_ = nullptr;
  iap::IntegritySnapshot snapshot_;
  int worker_count_ = 1;
  double hal_m_ = 10.0;
  double val_m_ = 20.0;
  bool ready_ = false;
  std::string begin_failure_reason_ = "provider_refresh_failed";
  iap::RollingSpatialRefreshDiagnostics begin_diagnostics_;
  iap::PredictorBatchDiagnostics last_diagnostics_;
};

class TimedRiskProvider final : public iap::RiskPredictionProvider {
 public:
  explicit TimedRiskProvider(iap::RiskPredictionProvider* provider)
      : provider_(provider) {}

  bool batchQuery(const std::vector<iap::RiskPredictionQuery>& queries,
                  std::vector<iap::RiskPredictionResult>* results) override {
    const auto start = std::chrono::steady_clock::now();
    const bool success = provider_ && provider_->batchQuery(queries, results);
    duration_ms_ += std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - start).count();
    return success;
  }

  double durationMs() const { return duration_ms_; }

 private:
  iap::RiskPredictionProvider* provider_ = nullptr;
  double duration_ms_ = 0.0;
};

}  // namespace

P0RiskGridRuntime::Config P0RiskGridRuntime::declareAndReadConfig(
    const rclcpp::Node::SharedPtr& node) {
  Config config;
  config.enable_risk_grid =
      node->declare_parameter<bool>("p0.enable_risk_grid", false);
  config.grid.resolution_m =
      node->declare_parameter<double>("p0.resolution_m", 0.75);
  config.grid.size_x_m =
      node->declare_parameter<double>("p0.size_x_m", 30.0);
  config.grid.size_y_m =
      node->declare_parameter<double>("p0.size_y_m", 30.0);
  config.grid.size_z_m =
      node->declare_parameter<double>("p0.size_z_m", 6.0);
  const Eigen::Vector3d fixed_origin(
      node->declare_parameter<double>(
          "p0.origin_x_m", std::numeric_limits<double>::quiet_NaN()),
      node->declare_parameter<double>(
          "p0.origin_y_m", std::numeric_limits<double>::quiet_NaN()),
      node->declare_parameter<double>(
          "p0.origin_z_m", std::numeric_limits<double>::quiet_NaN()));
  if (fixed_origin.allFinite()) {
    config.grid.use_fixed_origin = true;
    config.grid.fixed_origin_w = fixed_origin;
  }
  config.grid.horizons_s = node->declare_parameter<std::vector<double>>(
      "p0.horizons_s", std::vector<double>{0.0, 0.5, 1.0, 1.5, 2.0});
  config.grid.refresh_period_s =
      node->declare_parameter<double>("p0.refresh_period_s", 0.5);
  config.refresh_start_delay_s =
      node->declare_parameter<double>("p0.refresh_start_delay_s", 0.0);
  config.grid.stale_timeout_s =
      node->declare_parameter<double>("p0.stale_timeout_s", 1.0);
  config.grid.skip_occupied_voxels =
      node->declare_parameter<bool>("p0.skip_occupied_voxels",
                                    config.grid.skip_occupied_voxels);
  const std::string provider_cost_source = node->declare_parameter<std::string>(
      "p0.provider_cost_source", "legacy_safety_pl");
  if (provider_cost_source == "legacy_safety_pl") {
    config.grid.provider_cost_source =
        iap::RiskProviderCostSource::LEGACY_SAFETY_PL;
  } else if (provider_cost_source == "pre_conservative_fim_ratio") {
    config.grid.provider_cost_source =
        iap::RiskProviderCostSource::PRE_CONSERVATIVE_FIM_RATIO;
  } else {
    throw std::invalid_argument("unknown p0.provider_cost_source: " +
                                provider_cost_source);
  }
  config.grid.require_safety_ratio_below_one_for_cost =
      node->declare_parameter<bool>(
          "p0.require_safety_ratio_below_one_for_cost", false);
  config.grid.alert_limit_policy_id = node->declare_parameter<std::string>(
      "p0.alert_limit_policy_id", "legacy_unspecified");
  config.predictor_hal_m = node->declare_parameter<double>(
      "p0.alert_limit_h_m", 10.0);
  config.predictor_val_m = node->declare_parameter<double>(
      "p0.alert_limit_v_m", 20.0);
  config.grid.alert_limit_h_m = config.predictor_hal_m;
  config.grid.alert_limit_v_m = config.predictor_val_m;
  if (!std::isfinite(config.predictor_hal_m) ||
      !std::isfinite(config.predictor_val_m) ||
      config.predictor_hal_m <= 0.0 || config.predictor_val_m <= 0.0 ||
      config.grid.alert_limit_policy_id.empty()) {
    throw std::invalid_argument("invalid P0 alert-limit policy");
  }
  if (config.grid.alert_limit_policy_id == "fixed_hal10_val20_v1" &&
      (std::abs(config.predictor_hal_m - 10.0) > 1.0e-12 ||
       std::abs(config.predictor_val_m - 20.0) > 1.0e-12)) {
    throw std::invalid_argument(
        "fixed_hal10_val20_v1 requires HAL=10 m and VAL=20 m");
  }
  config.debug_metrics_enable =
      node->declare_parameter<bool>("p0.debug_metrics_enable", false);
  config.online_mapping_mode = node->declare_parameter<bool>(
      "p0.online_mapping_mode", false);
  config.grid.require_observed_support = config.online_mapping_mode;
  config.fit_grid_to_map_cloud = node->declare_parameter<bool>(
      "p0.fit_grid_to_map_cloud", false);
  config.odom_topic = node->declare_parameter<std::string>(
      "p0.odom_topic", "/drone_0_visual_slam/odom");
  config.integrity_topic =
      node->declare_parameter<std::string>("p0.integrity_topic",
                                           "/iap/integrity");
  config.range_meas_topic =
      node->declare_parameter<std::string>("p0.range_meas_topic",
                                           "/ublox_driver/range_meas");
  config.ephem_topic =
      node->declare_parameter<std::string>("p0.ephem_topic",
                                           "/ublox_driver/ephem");
  config.glo_ephem_topic =
      node->declare_parameter<std::string>("p0.glo_ephem_topic",
                                           "/ublox_driver/glo_ephem");
  config.receiver_lla_topic =
      node->declare_parameter<std::string>("p0.receiver_lla_topic",
                                           "/ublox_driver/receiver_lla");
  config.iono_topic =
      node->declare_parameter<std::string>("p0.iono_topic",
                                           "/ublox_driver/iono_params");
  config.map_topic = node->declare_parameter<std::string>(
      "p0.map_topic", "/map_generator/global_cloud");
  if (config.online_mapping_mode &&
      (config.fit_grid_to_map_cloud || !config.map_topic.empty())) {
    throw std::invalid_argument(
        "online P0 requires fit_grid_to_map_cloud=false and empty map_topic");
  }
  // Raw health is an acceptance artifact and must not inherit a node
  // namespace.  Keep declaring the old parameter for launch compatibility,
  // but deliberately publish the authoritative absolute topic.
  node->declare_parameter<std::string>("p0.health_topic",
                                       "/planning/risk_grid_health");
  config.health_topic = "/planning/risk_grid_health";
  config.gnss_epoch_max_age_s =
      node->declare_parameter<double>("p0.gnss_epoch_max_age_s", 2.0);
  config.predictor_source_mode = parsePredictorSourceMode(
      node->declare_parameter<std::string>("p0.predictor.source_mode",
                                           "fusion"));
  config.predictor_gnss_epoch_policy = parsePredictorGnssEpochPolicy(
      node->declare_parameter<std::string>(
          "p0.predictor.gnss_epoch_policy", "auto"));
  config.predictor_gnss_measured_epoch_support_radius_m =
      node->declare_parameter<double>(
          "p0.predictor.gnss_measured_epoch_support_radius_m", 0.0);
  if (!std::isfinite(
          config.predictor_gnss_measured_epoch_support_radius_m) ||
      config.predictor_gnss_measured_epoch_support_radius_m < 0.0) {
    throw std::invalid_argument(
        "invalid P0 measured GNSS epoch support radius");
  }
  config.predictor_gnss_measured_epoch_integrity_max_delta_s =
      node->declare_parameter<double>(
          "p0.predictor.gnss_measured_epoch_integrity_max_delta_s", 0.25);
  if (!std::isfinite(
          config.predictor_gnss_measured_epoch_integrity_max_delta_s) ||
      config.predictor_gnss_measured_epoch_integrity_max_delta_s < 0.0) {
    throw std::invalid_argument(
        "invalid P0 measured GNSS/integrity epoch alignment tolerance");
  }
  config.predictor_use_current_integrity_prior =
      node->declare_parameter<bool>(
          "p0.predictor.use_current_integrity_prior", true);
  config.predictor_conservative_max_with_gnss =
      node->declare_parameter<bool>(
          "p0.predictor.conservative_max_with_gnss", false);
  config.predictor_lidar_legacy_observability =
      node->declare_parameter<bool>(
          "p0.predictor.lidar_legacy_observability", true);
  config.predictor_lidar_fim_radius_m =
      node->declare_parameter<double>(
          "p0.predictor.lidar_fim_radius_m",
          config.predictor_lidar_fim_radius_m);
  config.predictor_sigma_grow_m_sqrt_s =
      node->declare_parameter<double>(
          "p0.predictor.sigma_grow_m_sqrt_s",
          config.predictor_sigma_grow_m_sqrt_s);
  config.predictor_requested_worker_count = static_cast<int>(std::max<int64_t>(1,
      node->declare_parameter<int>("p0.predictor.worker_count", 1)));
  config.predictor_effective_worker_count = config.predictor_requested_worker_count;
  config.p0_6_fixture.enabled =
      node->declare_parameter<bool>("p0_6.fixture.enabled", false);
  config.p0_6_fixture.name =
      node->declare_parameter<std::string>("p0_6.fixture.name", "");
  config.p0_6_fixture.x_min_m =
      node->declare_parameter<double>("p0_6.fixture.x_min",
                                      config.p0_6_fixture.x_min_m);
  config.p0_6_fixture.x_max_m =
      node->declare_parameter<double>("p0_6.fixture.x_max",
                                      config.p0_6_fixture.x_max_m);
  config.p0_6_fixture.y_min_m =
      node->declare_parameter<double>("p0_6.fixture.y_min",
                                      config.p0_6_fixture.y_min_m);
  config.p0_6_fixture.y_max_m =
      node->declare_parameter<double>("p0_6.fixture.y_max",
                                      config.p0_6_fixture.y_max_m);
  config.p0_6_fixture.z_min_m =
      node->declare_parameter<double>("p0_6.fixture.z_min",
                                      config.p0_6_fixture.z_min_m);
  config.p0_6_fixture.z_max_m =
      node->declare_parameter<double>("p0_6.fixture.z_max",
                                      config.p0_6_fixture.z_max_m);
  config.p0_6_fixture.raw_hpl_m =
      node->declare_parameter<double>("p0_6.fixture.raw_hpl_m",
                                      config.p0_6_fixture.raw_hpl_m);
  config.p0_6_fixture.raw_vpl_m =
      node->declare_parameter<double>("p0_6.fixture.raw_vpl_m",
                                      config.p0_6_fixture.raw_vpl_m);
  config.p0_6_fixture.raw_c_pi =
      node->declare_parameter<double>("p0_6.fixture.raw_c_pi",
                                      config.p0_6_fixture.raw_c_pi);
  config.p0_6_fixture.low_raw_cost_threshold =
      node->declare_parameter<double>(
          "p0_6.fixture.low_raw_cost_threshold",
          config.p0_6_fixture.low_raw_cost_threshold);
  if (config.p0_6_fixture.x_min_m > config.p0_6_fixture.x_max_m) {
    std::swap(config.p0_6_fixture.x_min_m,
              config.p0_6_fixture.x_max_m);
  }
  if (config.p0_6_fixture.y_min_m > config.p0_6_fixture.y_max_m) {
    std::swap(config.p0_6_fixture.y_min_m,
              config.p0_6_fixture.y_max_m);
  }
  if (config.p0_6_fixture.z_min_m > config.p0_6_fixture.z_max_m) {
    std::swap(config.p0_6_fixture.z_min_m,
              config.p0_6_fixture.z_max_m);
  }
  auto& p5_3_fixture = config.grid.p5_3_fixture;
  p5_3_fixture.enabled =
      node->declare_parameter<bool>("p5_3.fixture.enabled",
                                    p5_3_fixture.enabled);
  p5_3_fixture.name =
      node->declare_parameter<std::string>("p5_3.fixture.name",
                                           p5_3_fixture.name);
  p5_3_fixture.x_min_m =
      node->declare_parameter<double>("p5_3.fixture.x_min",
                                      p5_3_fixture.x_min_m);
  p5_3_fixture.x_max_m =
      node->declare_parameter<double>("p5_3.fixture.x_max",
                                      p5_3_fixture.x_max_m);
  p5_3_fixture.y_min_m =
      node->declare_parameter<double>("p5_3.fixture.y_min",
                                      p5_3_fixture.y_min_m);
  p5_3_fixture.y_max_m =
      node->declare_parameter<double>("p5_3.fixture.y_max",
                                      p5_3_fixture.y_max_m);
  p5_3_fixture.z_min_m =
      node->declare_parameter<double>("p5_3.fixture.z_min",
                                      p5_3_fixture.z_min_m);
  p5_3_fixture.z_max_m =
      node->declare_parameter<double>("p5_3.fixture.z_max",
                                      p5_3_fixture.z_max_m);
  p5_3_fixture.tau_min_s =
      node->declare_parameter<double>("p5_3.fixture.tau_min",
                                      p5_3_fixture.tau_min_s);
  p5_3_fixture.tau_max_s =
      node->declare_parameter<double>("p5_3.fixture.tau_max",
                                      p5_3_fixture.tau_max_s);
  p5_3_fixture.hpl_pred_m =
      node->declare_parameter<double>("p5_3.fixture.hpl_pred_m",
                                      p5_3_fixture.hpl_pred_m);
  p5_3_fixture.vpl_pred_m =
      node->declare_parameter<double>("p5_3.fixture.vpl_pred_m",
                                      p5_3_fixture.vpl_pred_m);
  if (p5_3_fixture.x_min_m > p5_3_fixture.x_max_m) {
    std::swap(p5_3_fixture.x_min_m, p5_3_fixture.x_max_m);
  }
  if (p5_3_fixture.y_min_m > p5_3_fixture.y_max_m) {
    std::swap(p5_3_fixture.y_min_m, p5_3_fixture.y_max_m);
  }
  if (p5_3_fixture.z_min_m > p5_3_fixture.z_max_m) {
    std::swap(p5_3_fixture.z_min_m, p5_3_fixture.z_max_m);
  }
  if (p5_3_fixture.tau_min_s > p5_3_fixture.tau_max_s) {
    std::swap(p5_3_fixture.tau_min_s, p5_3_fixture.tau_max_s);
  }
  auto& p5_4_fixture = config.grid.p5_4_fixture;
  p5_4_fixture.enabled =
      node->declare_parameter<bool>("p5_4.fixture.enabled",
                                    p5_4_fixture.enabled);
  p5_4_fixture.name =
      node->declare_parameter<std::string>("p5_4.fixture.name",
                                           p5_4_fixture.name);
  p5_4_fixture.x_min_m =
      node->declare_parameter<double>("p5_4.fixture.x_min",
                                      p5_4_fixture.x_min_m);
  p5_4_fixture.x_max_m =
      node->declare_parameter<double>("p5_4.fixture.x_max",
                                      p5_4_fixture.x_max_m);
  p5_4_fixture.y_min_m =
      node->declare_parameter<double>("p5_4.fixture.y_min",
                                      p5_4_fixture.y_min_m);
  p5_4_fixture.y_max_m =
      node->declare_parameter<double>("p5_4.fixture.y_max",
                                      p5_4_fixture.y_max_m);
  p5_4_fixture.z_min_m =
      node->declare_parameter<double>("p5_4.fixture.z_min",
                                      p5_4_fixture.z_min_m);
  p5_4_fixture.z_max_m =
      node->declare_parameter<double>("p5_4.fixture.z_max",
                                      p5_4_fixture.z_max_m);
  p5_4_fixture.tau_min_s =
      node->declare_parameter<double>("p5_4.fixture.tau_min",
                                      p5_4_fixture.tau_min_s);
  p5_4_fixture.tau_max_s =
      node->declare_parameter<double>("p5_4.fixture.tau_max",
                                      p5_4_fixture.tau_max_s);
  p5_4_fixture.hpl_pred_m =
      node->declare_parameter<double>("p5_4.fixture.hpl_pred_m",
                                      p5_4_fixture.hpl_pred_m);
  p5_4_fixture.vpl_pred_m =
      node->declare_parameter<double>("p5_4.fixture.vpl_pred_m",
                                      p5_4_fixture.vpl_pred_m);
  if (p5_4_fixture.x_min_m > p5_4_fixture.x_max_m) {
    std::swap(p5_4_fixture.x_min_m, p5_4_fixture.x_max_m);
  }
  if (p5_4_fixture.y_min_m > p5_4_fixture.y_max_m) {
    std::swap(p5_4_fixture.y_min_m, p5_4_fixture.y_max_m);
  }
  if (p5_4_fixture.z_min_m > p5_4_fixture.z_max_m) {
    std::swap(p5_4_fixture.z_min_m, p5_4_fixture.z_max_m);
  }
  if (p5_4_fixture.tau_min_s > p5_4_fixture.tau_max_s) {
    std::swap(p5_4_fixture.tau_min_s, p5_4_fixture.tau_max_s);
  }
  auto& p5_6_fixture = config.grid.p5_6_fixture;
  p5_6_fixture.enabled =
      node->declare_parameter<bool>("p5_6.fixture.enabled",
                                    p5_6_fixture.enabled);
  p5_6_fixture.name =
      node->declare_parameter<std::string>("p5_6.fixture.name",
                                           p5_6_fixture.name);
  p5_6_fixture.x_min_m =
      node->declare_parameter<double>("p5_6.fixture.x_min",
                                      p5_6_fixture.x_min_m);
  p5_6_fixture.x_max_m =
      node->declare_parameter<double>("p5_6.fixture.x_max",
                                      p5_6_fixture.x_max_m);
  p5_6_fixture.y_min_m =
      node->declare_parameter<double>("p5_6.fixture.y_min",
                                      p5_6_fixture.y_min_m);
  p5_6_fixture.y_max_m =
      node->declare_parameter<double>("p5_6.fixture.y_max",
                                      p5_6_fixture.y_max_m);
  p5_6_fixture.z_min_m =
      node->declare_parameter<double>("p5_6.fixture.z_min",
                                      p5_6_fixture.z_min_m);
  p5_6_fixture.z_max_m =
      node->declare_parameter<double>("p5_6.fixture.z_max",
                                      p5_6_fixture.z_max_m);
  p5_6_fixture.tau_min_s =
      node->declare_parameter<double>("p5_6.fixture.tau_min",
                                      p5_6_fixture.tau_min_s);
  p5_6_fixture.tau_max_s =
      node->declare_parameter<double>("p5_6.fixture.tau_max",
                                      p5_6_fixture.tau_max_s);
  if (p5_6_fixture.x_min_m > p5_6_fixture.x_max_m) {
    std::swap(p5_6_fixture.x_min_m, p5_6_fixture.x_max_m);
  }
  if (p5_6_fixture.y_min_m > p5_6_fixture.y_max_m) {
    std::swap(p5_6_fixture.y_min_m, p5_6_fixture.y_max_m);
  }
  if (p5_6_fixture.z_min_m > p5_6_fixture.z_max_m) {
    std::swap(p5_6_fixture.z_min_m, p5_6_fixture.z_max_m);
  }
  if (p5_6_fixture.tau_min_s > p5_6_fixture.tau_max_s) {
    std::swap(p5_6_fixture.tau_min_s, p5_6_fixture.tau_max_s);
  }
  auto& p5_7_fixture = config.grid.p5_7_fixture;
  p5_7_fixture.enabled =
      node->declare_parameter<bool>("p5_7.fixture.enabled",
                                    p5_7_fixture.enabled);
  p5_7_fixture.effective_enabled =
      node->declare_parameter<bool>("p5_7.fixture.effective_enabled",
                                    p5_7_fixture.effective_enabled);
  p5_7_fixture.name =
      node->declare_parameter<std::string>("p5_7.fixture.name",
                                           p5_7_fixture.name);
  p5_7_fixture.x_min_m =
      node->declare_parameter<double>("p5_7.fixture.x_min",
                                      p5_7_fixture.x_min_m);
  p5_7_fixture.x_max_m =
      node->declare_parameter<double>("p5_7.fixture.x_max",
                                      p5_7_fixture.x_max_m);
  p5_7_fixture.y_min_m =
      node->declare_parameter<double>("p5_7.fixture.y_min",
                                      p5_7_fixture.y_min_m);
  p5_7_fixture.y_max_m =
      node->declare_parameter<double>("p5_7.fixture.y_max",
                                      p5_7_fixture.y_max_m);
  p5_7_fixture.z_min_m =
      node->declare_parameter<double>("p5_7.fixture.z_min",
                                      p5_7_fixture.z_min_m);
  p5_7_fixture.z_max_m =
      node->declare_parameter<double>("p5_7.fixture.z_max",
                                      p5_7_fixture.z_max_m);
  p5_7_fixture.tau_min_s =
      node->declare_parameter<double>("p5_7.fixture.tau_min",
                                      p5_7_fixture.tau_min_s);
  p5_7_fixture.tau_max_s =
      node->declare_parameter<double>("p5_7.fixture.tau_max",
                                      p5_7_fixture.tau_max_s);
  p5_7_fixture.hpl_pred_m =
      node->declare_parameter<double>("p5_7.fixture.hpl_pred_m",
                                      p5_7_fixture.hpl_pred_m);
  p5_7_fixture.vpl_pred_m =
      node->declare_parameter<double>("p5_7.fixture.vpl_pred_m",
                                      p5_7_fixture.vpl_pred_m);
  if (p5_7_fixture.x_min_m > p5_7_fixture.x_max_m) {
    std::swap(p5_7_fixture.x_min_m, p5_7_fixture.x_max_m);
  }
  if (p5_7_fixture.y_min_m > p5_7_fixture.y_max_m) {
    std::swap(p5_7_fixture.y_min_m, p5_7_fixture.y_max_m);
  }
  if (p5_7_fixture.z_min_m > p5_7_fixture.z_max_m) {
    std::swap(p5_7_fixture.z_min_m, p5_7_fixture.z_max_m);
  }
  if (p5_7_fixture.tau_min_s > p5_7_fixture.tau_max_s) {
    std::swap(p5_7_fixture.tau_min_s, p5_7_fixture.tau_max_s);
  }
  return config;
}

std::unique_ptr<P0RiskGridRuntime> P0RiskGridRuntime::createIfEnabled(
    const rclcpp::Node::SharedPtr& node) {
  Config config = declareAndReadConfig(node);
  if (!config.enable_risk_grid) {
    return nullptr;
  }
  return std::make_unique<P0RiskGridRuntime>(node, std::move(config));
}

P0RiskGridRuntime::P0RiskGridRuntime(
    rclcpp::Node::SharedPtr node,
    Config config,
    std::unique_ptr<iap::RiskPredictionProvider> provider)
    : node_(std::move(node)),
      config_(std::move(config)),
      risk_grid_(config_.grid),
      provider_(std::move(provider)) {
  createRosInterfaces();
}

iap::RiskGridHealth P0RiskGridRuntime::health() const {
  const double now_s = liveNowSeconds();
  return addLidarPredictorInputHealth(
      std::isfinite(now_s) ? risk_grid_.health(now_s)
                           : risk_grid_.health());
}

std::shared_ptr<const P0PlanningSnapshot>
P0RiskGridRuntime::acquirePlanningSnapshot() const {
  std::lock_guard<std::mutex> lock(planning_snapshot_mutex_);
  return planning_snapshot_;
}

std::shared_ptr<const iap::RiskGridSnapshot>
P0RiskGridRuntime::acquireSnapshot() const {
  const auto planning = acquirePlanningSnapshot();
  return planning ? planning->risk : nullptr;
}

bool P0RiskGridRuntime::refreshOnceForTest() {
  refreshTimerCallback();
  std::lock_guard<std::mutex> lock(health_state_mutex_);
  return last_refresh_succeeded_;
}

void P0RiskGridRuntime::setOccupancyPredicate(
    iap::RiskGridMap::OccupancyPredicate predicate) {
  occupancy_predicate_ = std::move(predicate);
}

void P0RiskGridRuntime::setOccupancyDiagnosticQuery(
    iap::RiskGridMap::OccupancyDiagnosticQuery query) {
  occupancy_diagnostic_query_ = std::move(query);
}

void P0RiskGridRuntime::setOccupancyDiagnosticQueryFactory(
    std::function<iap::RiskGridMap::OccupancyDiagnosticQuery()> factory) {
  occupancy_diagnostic_query_factory_ = std::move(factory);
}

void P0RiskGridRuntime::setOccupancyEpochFactory(
    std::function<P0OccupancyEpochCapture()> factory) {
  occupancy_epoch_factory_ = std::move(factory);
}

bool P0RiskGridRuntime::p0_6_fixture_occupied(
    const Eigen::Vector3d& pos) const {
  const auto& fixture = config_.p0_6_fixture;
  if (!fixture.enabled || fixture.name != "occupied_overlap_box_v1" ||
      !pos.allFinite()) {
    return false;
  }
  return pos.x() >= fixture.x_min_m && pos.x() <= fixture.x_max_m &&
         pos.y() >= fixture.y_min_m && pos.y() <= fixture.y_max_m &&
         pos.z() >= fixture.z_min_m && pos.z() <= fixture.z_max_m;
}

iap::RiskGridMap::OccupancyPredicate
P0RiskGridRuntime::combinedOccupancyPredicate() const {
  const bool fixture_enabled =
      config_.p0_6_fixture.enabled &&
      config_.p0_6_fixture.name == "occupied_overlap_box_v1";
  if (!fixture_enabled) {
    return occupancy_predicate_;
  }
  return [this](const Eigen::Vector3d& pos) {
    if (occupancy_predicate_ && occupancy_predicate_(pos)) {
      return true;
    }
    return p0_6_fixture_occupied(pos);
  };
}

iap::RiskGridMap::OccupancyDiagnosticQuery
P0RiskGridRuntime::combinedOccupancyDiagnosticQuery(
    iap::RiskGridMap::OccupancyDiagnosticQuery base_query) const {
  auto query = std::move(base_query);
  if (!query) {
    query = occupancy_diagnostic_query_factory_
        ? occupancy_diagnostic_query_factory_()
        : occupancy_diagnostic_query_;
  }
  if (!query && occupancy_diagnostic_query_factory_) {
    query = [](const Eigen::Vector3d&) {
      iap::RiskOccupancyDiagnostic diagnostic;
      diagnostic.source = "occupancy_snapshot_unavailable";
      return diagnostic;
    };
  }
  if (!query) {
    return {};
  }
  const bool fixture_enabled =
      config_.p0_6_fixture.enabled &&
      config_.p0_6_fixture.name == "occupied_overlap_box_v1";
  if (!fixture_enabled) {
    return query;
  }
  return [this, query = std::move(query)](const Eigen::Vector3d& pos) {
    auto diagnostic = query(pos);
    if (p0_6_fixture_occupied(pos)) {
      diagnostic.available = true;
      diagnostic.raw_occupied = true;
      diagnostic.inflated_occupied = true;
      diagnostic.source = "p0_6_fixture";
    }
    return diagnostic;
  };
}

void P0RiskGridRuntime::createRosInterfaces() {
  if (!node_ || !config_.enable_risk_grid) {
    return;
  }
  input_callback_group_ =
      node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  predictor_input_callback_group_ =
      node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  map_input_callback_group_ =
      node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  refresh_callback_group_ =
      node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  health_callback_group_ =
      node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  rclcpp::SubscriptionOptions subscription_options;
  subscription_options.callback_group = input_callback_group_;
  rclcpp::SubscriptionOptions predictor_subscription_options;
  predictor_subscription_options.callback_group =
      predictor_input_callback_group_;
  rclcpp::SubscriptionOptions map_subscription_options;
  map_subscription_options.callback_group = map_input_callback_group_;
  const rclcpp::QoS qos(50);
  odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
      config_.odom_topic, qos,
      [this](const nav_msgs::msg::Odometry::ConstSharedPtr msg) {
        odomCallback(msg);
      },
      subscription_options);
  integrity_sub_ = node_->create_subscription<iap::msg::IntegrityReport>(
      config_.integrity_topic, qos,
      [this](const iap::msg::IntegrityReport::ConstSharedPtr msg) {
        integrityCallback(msg);
      },
      subscription_options);
  range_sub_ = node_->create_subscription<gnss_comm::msg::GnssMeasMsg>(
      config_.range_meas_topic, qos,
      [this](const gnss_comm::msg::GnssMeasMsg::ConstSharedPtr msg) {
        rangeCallback(msg);
      },
      predictor_subscription_options);
  ephem_sub_ = node_->create_subscription<gnss_comm::msg::GnssEphemMsg>(
      config_.ephem_topic, qos,
          [this](const gnss_comm::msg::GnssEphemMsg::ConstSharedPtr msg) {
            ephemCallback(msg);
          },
          predictor_subscription_options);
  glo_ephem_sub_ =
      node_->create_subscription<gnss_comm::msg::GnssGloEphemMsg>(
          config_.glo_ephem_topic, qos,
          [this](const gnss_comm::msg::GnssGloEphemMsg::ConstSharedPtr msg) {
            gloEphemCallback(msg);
          },
          predictor_subscription_options);
  receiver_lla_sub_ =
      node_->create_subscription<sensor_msgs::msg::NavSatFix>(
          config_.receiver_lla_topic, qos,
          [this](const sensor_msgs::msg::NavSatFix::ConstSharedPtr msg) {
            receiverLlaCallback(msg);
          },
          predictor_subscription_options);
  iono_sub_ =
      node_->create_subscription<gnss_comm::msg::GnssIonosphereParameter>(
          config_.iono_topic, qos,
          [this](const gnss_comm::msg::GnssIonosphereParameter::ConstSharedPtr msg) {
            ionoCallback(msg);
          },
          predictor_subscription_options);
  if (!config_.online_mapping_mode) {
    cloud_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
        config_.map_topic, rclcpp::QoS(2).transient_local().reliable(),
        [this](const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
          cloudCallback(msg);
        },
        map_subscription_options);
  }

  // Do not couple the machine-readable health contract to RViz/debug flags.
  health_pub_ = node_->create_publisher<std_msgs::msg::String>(
      "/planning/risk_grid_health", 10);
  safety_viz_ = std::make_shared<SafetyRvizPublisher>(
      node_, SafetyRvizPublisher::declareAndReadConfig(node_));
  const double period_s =
      std::max(0.001, config_.grid.refresh_period_s);
  const double refresh_start_delay_s =
      std::max(0.0, config_.refresh_start_delay_s);
  auto start_periodic_refresh = [this, period_s]() {
    refresh_timer_ = node_->create_wall_timer(
        std::chrono::duration<double>(period_s),
        [this]() { refreshTimerCallback(); }, refresh_callback_group_);
    refreshTimerCallback();
  };
  {
    std::lock_guard<std::mutex> lock(health_state_mutex_);
    next_refresh_scheduled_steady_s_ =
        steadyNowSeconds() + period_s + refresh_start_delay_s;
  }
  if (refresh_start_delay_s > 0.0) {
    refresh_start_timer_ = node_->create_wall_timer(
        std::chrono::duration<double>(period_s + refresh_start_delay_s),
        [this, start_periodic_refresh]() {
          refresh_start_timer_->cancel();
          start_periodic_refresh();
        },
        refresh_callback_group_);
  } else {
    refresh_timer_ = node_->create_wall_timer(
        std::chrono::duration<double>(period_s),
        [this]() { refreshTimerCallback(); }, refresh_callback_group_);
  }
  // Health must remain observable while a full grid refresh is evaluating a
  // large predictor batch.  It intentionally publishes the latest snapshot
  // state rather than waiting for that batch to finish.
  health_timer_ = node_->create_wall_timer(
      std::chrono::duration<double>(period_s),
      [this]() { healthTimerCallback(); },
      health_callback_group_);
}

void P0RiskGridRuntime::healthTimerCallback() {
  if (!config_.enable_risk_grid) {
    return;
  }
  const auto callback_start = std::chrono::steady_clock::now();
  const double callback_steady_s = steadyNowSeconds();
  const double now_s = liveNowSeconds();
  {
    std::lock_guard<std::mutex> lock(health_state_mutex_);
    last_health_callback_stamp_s_ = now_s;
    last_health_callback_steady_s_ = callback_steady_s;
    ++health_callback_count_;
    // A wall timer has no exposed scheduled-fire stamp.  The callback group
    // is reentrant, so this remains a conservative observable queue delay.
    last_health_callback_queue_delay_ms_ = 0.0;
  }
  publishHealth(std::isfinite(now_s) ? risk_grid_.health(now_s)
                                     : risk_grid_.health(), now_s);
  const double duration_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - callback_start).count();
  {
    std::lock_guard<std::mutex> lock(health_state_mutex_);
    last_health_callback_duration_ms_ = duration_ms;
  }
}

bool P0RiskGridRuntime::geometryMatchesRiskOverlay(
    const iap::PlanningLatticeGeometry& geometry,
    const iap::RiskGridMapParams& risk_grid,
    const int required_ego_voxels_per_axis) {
  if (!geometry.valid() || required_ego_voxels_per_axis <= 0 ||
      !risk_grid.use_fixed_origin) {
    return false;
  }
  const Eigen::Vector3d configured_extent(
      risk_grid.size_x_m, risk_grid.size_y_m, risk_grid.size_z_m);
  const double ratio = risk_grid.resolution_m / geometry.resolution_m;
  return std::isfinite(ratio) &&
      std::abs(ratio - static_cast<double>(required_ego_voxels_per_axis)) <=
          1.0e-9 &&
      geometry.frame_id == risk_grid.frame_id &&
      geometry.origin_w.isApprox(risk_grid.fixed_origin_w, 1.0e-9) &&
      geometry.extent_m.isApprox(configured_extent, 1.0e-9);
}

void P0RiskGridRuntime::refreshTimerCallback() {
  if (!config_.enable_risk_grid) {
    return;
  }
  const auto refresh_start = std::chrono::steady_clock::now();
  const double refresh_start_steady_s = steadyNowSeconds();
  double now_s = liveNowSeconds();
  std::optional<P0OccupancyEpoch> occupancy_epoch;
  P0OccupancyEpochCaptureStatus occupancy_capture_status =
      P0OccupancyEpochCaptureStatus::VALID;
  const bool occupancy_capture_attempted =
      static_cast<bool>(occupancy_epoch_factory_);
  if (occupancy_capture_attempted) {
    P0OccupancyEpochCapture capture = occupancy_epoch_factory_();
    occupancy_capture_status = capture.status;
    occupancy_epoch = std::move(capture.epoch);
  }
  if (config_.online_mapping_mode && occupancy_epoch.has_value()) {
    const auto& geometry = occupancy_epoch->geometry;
    const bool geometry_matches = geometryMatchesRiskOverlay(
        geometry, config_.grid, 5);
    if (!geometry_matches) {
      occupancy_capture_status =
          P0OccupancyEpochCaptureStatus::ADAPTER_INVALID;
      occupancy_epoch.reset();
    } else {
      if (!online_geometry_bound_) {
        config_.grid.geometry_id = geometry.geometry_id;
        std::string configure_reason;
        if (!risk_grid_.configure(config_.grid, &configure_reason)) {
          occupancy_capture_status =
              P0OccupancyEpochCaptureStatus::ADAPTER_INVALID;
          occupancy_epoch.reset();
        } else {
          online_geometry_bound_ = true;
        }
      }
      if (occupancy_epoch.has_value()) {
        // The adapter's immutable LOS grid does not expose its point vector;
        // reconstruct it from the normalized occupied identity on the same
        // frozen lattice, never from a simulator map topic.
        auto points = std::make_shared<std::vector<Eigen::Vector3d>>();
        if (occupancy_epoch->raw_identity) {
          points->reserve(occupancy_epoch->raw_identity->keys().size());
          for (const auto& key : occupancy_epoch->raw_identity->keys()) {
            points->push_back(
                occupancy_epoch->raw_identity->latticeOrigin() +
                (Eigen::Vector3d(key.x, key.y, key.z) +
                 Eigen::Vector3d::Constant(0.5)) *
                    occupancy_epoch->raw_identity->resolutionM());
          }
        }
        iap::LidarFimPrimitiveGenerationDiagnostics diagnostics;
        auto primitives = iap::make_lidar_fim_primitives(
            *points, nullptr, iap::LidarFimPrimitiveGenerationParams{},
            &diagnostics);
        {
          std::lock_guard<std::mutex> lock(lidar_predictor_input_mutex_);
          latest_lidar_map_points_ = points;
          latest_lidar_fim_primitives_ = primitives;
          latest_lidar_fim_diagnostics_ = diagnostics;
          latest_lidar_map_point_count_ = points->size();
          latest_lidar_fim_primitive_count_ =
              primitives ? primitives->size() : 0u;
          latest_lidar_fim_valid_normal_count_ =
              static_cast<std::size_t>(
                  std::max(0, diagnostics.lidar_pca_valid_normals));
          latest_lidar_fim_fallback_reason_ = diagnostics.fallback_reason;
          latest_lidar_generation_ = occupancy_epoch->generation;
          latest_lidar_stamp_ = occupancy_epoch->cloud_stamp_s;
        }
        {
          std::lock_guard<std::mutex> lock(health_state_mutex_);
          map_seen_ = true;
          latest_map_stamp_ = occupancy_epoch->cloud_stamp_s;
        }
      }
    }
  }
  const std::string pre_build_failure = snapshotFailureReason(now_s);
  const InputReadiness refresh_input_readiness = inputReadiness(now_s);
  const iap::RiskGridHealth active_health_at_start =
      std::isfinite(now_s) ? risk_grid_.health(now_s) : risk_grid_.health();
  {
    std::lock_guard<std::mutex> health_lock(health_state_mutex_);
    last_refresh_stamp_s_ = now_s;
    last_refresh_scheduled_steady_s_ = next_refresh_scheduled_steady_s_;
    last_refresh_queue_delay_ms_ = std::isfinite(next_refresh_scheduled_steady_s_)
        ? std::max(0.0, 1000.0 * (refresh_start_steady_s -
                                  next_refresh_scheduled_steady_s_))
        : std::numeric_limits<double>::quiet_NaN();
    const double period_s = std::max(0.001, config_.grid.refresh_period_s);
    if (!std::isfinite(next_refresh_scheduled_steady_s_))
      next_refresh_scheduled_steady_s_ = refresh_start_steady_s + period_s;
    while (next_refresh_scheduled_steady_s_ <= refresh_start_steady_s)
      next_refresh_scheduled_steady_s_ += period_s;
    last_refresh_start_stamp_s_ = now_s;
    last_refresh_start_steady_s_ = refresh_start_steady_s;
    last_snapshot_available_ = false;
    last_refresh_succeeded_ = false;
    last_snapshot_failure_reason_ = pre_build_failure;
    last_refresh_query_count_ = 0;
    last_predictor_unique_positions_ = 0;
    last_predictor_lidar_evaluations_ = 0;
    last_predictor_lidar_cache_hits_ = 0;
    last_predictor_spatial_advisory_recompute_count_ = 0;
    last_predictor_spatial_advisory_reuse_count_ = 0;
    last_predictor_gnss_advisory_invocation_count_ = 0;
    last_predictor_lidar_advisory_invocation_count_ = 0;
    last_predictor_horizon_fusion_count_ = 0;
    last_rolling_spatial_diagnostics_ = {};
    last_provider_batch_duration_ms_ =
        std::numeric_limits<double>::quiet_NaN();
    last_refresh_elapsed_ms_ = std::numeric_limits<double>::quiet_NaN();
    last_generation_interval_ms_ = std::numeric_limits<double>::quiet_NaN();
    last_refresh_end_stamp_s_ = std::numeric_limits<double>::quiet_NaN();
    last_refresh_end_steady_s_ = std::numeric_limits<double>::quiet_NaN();
    refresh_evidence_ = {};
    refresh_evidence_.refresh_attempt_id = next_refresh_attempt_id_++;
    refresh_evidence_.state = RefreshEvidenceState::IN_PROGRESS;
    refresh_evidence_.previous_successful_generation_id =
        last_successful_generation_id_;
    refresh_evidence_.publication = healthPublicationStateSnapshot();
    refresh_evidence_.health = active_health_at_start;
    refresh_evidence_.health.provider_query_count = 0;
    refresh_evidence_.health.occupied_skip_count = 0;
    refresh_evidence_.snapshot_failure_reason = "none";
    refresh_input_readiness_ = refresh_input_readiness;
  }
  if (!std::isfinite(now_s) || now_s <= 0.0) {
    risk_grid_.markRefreshFailure(now_s, "message_stamp_unavailable");
    const double refresh_end_stamp_s = liveNowSeconds();
    {
      std::lock_guard<std::mutex> health_lock(health_state_mutex_);
      last_refresh_elapsed_ms_ = std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - refresh_start).count();
      last_refresh_end_stamp_s_ = refresh_end_stamp_s;
      last_refresh_end_steady_s_ = steadyNowSeconds();
    }
    completeRefreshEvidence(risk_grid_.health(), now_s, false);
    publishHealth(risk_grid_.health(), now_s);
    return;
  }
  const auto fail_semantic_refresh = [&](const P0SemanticFailure failure) {
    const std::string failure_reason = semanticFailureReason(failure);
    risk_grid_.markRefreshFailure(now_s, failure_reason);
    const double refresh_end_stamp_s = liveNowSeconds();
    {
      std::lock_guard<std::mutex> health_lock(health_state_mutex_);
      last_snapshot_failure_reason_ = failure_reason;
      last_refresh_succeeded_ = false;
      last_refresh_elapsed_ms_ = std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - refresh_start).count();
      last_refresh_end_stamp_s_ = refresh_end_stamp_s;
      last_refresh_end_steady_s_ = steadyNowSeconds();
    }
    const iap::RiskGridHealth failed_health = risk_grid_.health(now_s);
    completeRefreshEvidence(failed_health, now_s, false);
    publishHealth(failed_health, now_s);
  };
  const bool production_predictor = provider_ == nullptr;
  if (production_predictor && config_.predictor_use_current_integrity_prior) {
    const InputReadiness readiness = inputReadiness(now_s);
    if (readiness.current_integrity_seen &&
        readiness.current_integrity_valid &&
        !readiness.current_integrity_fresh &&
        std::isfinite(readiness.current_integrity_stamp_s)) {
      fail_semantic_refresh(P0SemanticFailure::STALE_COVARIANCE_GROWTH_PRIOR);
      return;
    }
  }
  iap::IntegritySnapshot snapshot;
  PredictorSourceCapture captured_predictor_sources;
  InputReadiness captured_input_readiness;
  const bool snapshot_built = buildSnapshot(
      now_s, &snapshot, &captured_predictor_sources,
      &captured_input_readiness);
  {
    std::lock_guard<std::mutex> health_lock(health_state_mutex_);
    refresh_input_readiness_ = captured_input_readiness;
  }
  if (!snapshot_built) {
    const bool invalid_current_provenance =
        production_predictor &&
        (captured_predictor_sources.current_generation == 0u ||
         !std::isfinite(captured_predictor_sources.current_stamp));
    const bool invalid_gnss_provenance =
        production_predictor &&
        config_.predictor_source_mode != iap::PredictorSourceMode::LidarOnly &&
        config_.predictor_gnss_epoch_policy !=
            iap::PredictorGnssEpochPolicy::Disabled &&
        captured_predictor_sources.gnss_epoch_generation != 0u &&
        !std::isfinite(captured_predictor_sources.gnss_epoch_stamp);
    const std::string failure_reason =
        invalid_current_provenance
            ? "invalid_current_provenance"
            : invalid_gnss_provenance ? "invalid_gnss_epoch_identity"
                                      : "snapshot_unavailable";
    risk_grid_.markRefreshFailure(now_s, failure_reason);
    const double refresh_end_stamp_s = liveNowSeconds();
    {
      std::lock_guard<std::mutex> health_lock(health_state_mutex_);
      last_refresh_elapsed_ms_ = std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - refresh_start).count();
      last_refresh_end_stamp_s_ = refresh_end_stamp_s;
      last_refresh_end_steady_s_ = steadyNowSeconds();
      if (invalid_current_provenance || invalid_gnss_provenance) {
        last_snapshot_failure_reason_ = failure_reason;
        last_rolling_spatial_diagnostics_.invalid_source_provenance_count =
            1u;
        last_rolling_spatial_diagnostics_.invalidation_reason =
            iap::RollingSpatialInvalidationReason::SourceProvenanceInvalid;
      } else if (pre_build_failure == "none") {
        last_snapshot_failure_reason_ = "snapshot_builder_invalid";
      }
    }
    const iap::RiskGridHealth failed_health = risk_grid_.health(now_s);
    completeRefreshEvidence(failed_health, now_s, false);
    publishHealth(failed_health, now_s);
    return;
  }
  {
    std::lock_guard<std::mutex> health_lock(health_state_mutex_);
    last_snapshot_available_ = true;
    last_snapshot_failure_reason_ = "none";
  }

  if (occupancy_capture_attempted) {
    if (occupancy_capture_status ==
        P0OccupancyEpochCaptureStatus::SNAPSHOT_UNAVAILABLE) {
      fail_semantic_refresh(
          P0SemanticFailure::OCCUPANCY_SNAPSHOT_UNAVAILABLE);
      return;
    }
    if (occupancy_capture_status ==
            P0OccupancyEpochCaptureStatus::ADAPTER_INVALID ||
        !occupancy_epoch.has_value()) {
      fail_semantic_refresh(P0SemanticFailure::OCCUPANCY_LOS_ADAPTER_INVALID);
      return;
    }
    if (!occupancy_epoch->diagnostic_query ||
        !occupancy_epoch->los_owner || !occupancy_epoch->source_owner ||
        !occupancy_epoch->live_source_owner ||
        !occupancy_epoch->live_generation ||
        occupancy_epoch->generation == 0u) {
      fail_semantic_refresh(P0SemanticFailure::OCCUPANCY_LOS_ADAPTER_INVALID);
      return;
    }
    if (occupancy_epoch->frame_id != config_.grid.frame_id ||
        occupancy_epoch->frame_id != "map") {
      fail_semantic_refresh(P0SemanticFailure::OCCUPANCY_FRAME_MISMATCH);
      return;
    }
    const double occupancy_age_s = now_s - occupancy_epoch->cloud_stamp_s;
    if (!std::isfinite(occupancy_epoch->cloud_stamp_s) ||
        !std::isfinite(occupancy_age_s) || occupancy_age_s < 0.0 ||
        (config_.grid.stale_timeout_s >= 0.0 &&
         occupancy_age_s > config_.grid.stale_timeout_s)) {
      RCLCPP_WARN(
          node_->get_logger(),
          "[p0] rejecting occupancy epoch generation=%lu stamp=%.9f "
          "transaction_now=%.9f age=%.6f stale_timeout=%.6f",
          static_cast<unsigned long>(occupancy_epoch->generation),
          occupancy_epoch->cloud_stamp_s, now_s, occupancy_age_s,
          config_.grid.stale_timeout_s);
      fail_semantic_refresh(P0SemanticFailure::OCCUPANCY_STALE);
      return;
    }
  }

  if (production_predictor && !occupancy_epoch.has_value()) {
    fail_semantic_refresh(P0SemanticFailure::OCCUPANCY_SNAPSHOT_UNAVAILABLE);
    return;
  }
  if (production_predictor &&
      (!std::isfinite(config_.predictor_sigma_grow_m_sqrt_s) ||
       config_.predictor_sigma_grow_m_sqrt_s < 0.0)) {
    fail_semantic_refresh(
        P0SemanticFailure::INVALID_COVARIANCE_GROWTH_PARAMETER);
    return;
  }
  if (production_predictor && !snapshot.has_lambda_base) {
    fail_semantic_refresh(
        config_.predictor_use_current_integrity_prior
            ? P0SemanticFailure::INVALID_COVARIANCE_GROWTH_PRIOR
            : P0SemanticFailure::MISSING_COVARIANCE_GROWTH_PRIOR);
    return;
  }
  if (production_predictor && !finiteSpd(snapshot.lambda_base_pos)) {
    fail_semantic_refresh(P0SemanticFailure::INVALID_COVARIANCE_GROWTH_PRIOR);
    return;
  }

  std::unique_ptr<iap::RiskPredictionProvider> local_provider;
  iap::RiskPredictionProvider* provider = provider_.get();
  PredictorModuleRiskProvider* predictor_provider = nullptr;
  bool validate_gnss_spatial_source = false;
  bool validate_lidar_spatial_source = false;
  bool validate_lidar_legacy_source = false;
  bool active_gnss_occupancy_content = false;
  uint64_t candidate_occupancy_content_identity = 0;
  std::shared_ptr<const std::vector<Eigen::Vector3d>>
      captured_lidar_map_points;
  std::shared_ptr<const std::vector<iap::LidarFimPrimitive>>
      captured_lidar_fim_primitives;
  std::shared_ptr<iap::PredictorModule> forward_risk_module;
  std::shared_ptr<const iap::LocalOccupancyGrid>
      forward_risk_occupancy_owner;
  double forward_gnss_support_ray_length_m =
      std::numeric_limits<double>::quiet_NaN();
  bool forward_gnss_hard_occlusion = false;
  uint64_t captured_lidar_generation = 0;
  double captured_lidar_stamp =
      std::numeric_limits<double>::quiet_NaN();
  if (provider == nullptr) {
    iap::PredictorParams predictor_params;
    predictor_params.freshness.enabled = true;
    predictor_params.freshness.max_odom_age_s =
        config_.grid.stale_timeout_s;
    predictor_params.freshness.max_integrity_age_s =
        config_.grid.stale_timeout_s;
    predictor_params.freshness.max_gnss_age_s =
        config_.gnss_epoch_max_age_s;
    predictor_params.freshness.max_snapshot_age_s =
        config_.grid.stale_timeout_s;
    predictor_params.source_mode = config_.predictor_source_mode;
    forward_gnss_hard_occlusion =
        predictor_params.gnss.visibility_params.hard_occlusion;
    forward_gnss_support_ray_length_m = forward_gnss_hard_occlusion
        ? predictor_params.gnss.visibility_params.occ_range
        : predictor_params.gnss.visibility_params.occ_L;
    predictor_params.gnss_epoch_policy =
        config_.predictor_gnss_epoch_policy;
    predictor_params.gnss.measured_epoch_support_radius_m =
        config_.predictor_gnss_measured_epoch_support_radius_m;
    predictor_params.gnss.measured_epoch_integrity_max_delta_s =
        config_.predictor_gnss_measured_epoch_integrity_max_delta_s;
    predictor_params.fusion.conservative_max_with_gnss =
        config_.predictor_conservative_max_with_gnss;
    predictor_params.lidar.enable_legacy_observability =
        config_.predictor_lidar_legacy_observability;
    predictor_params.covariance_growth.sigma_grow_m_sqrt_s =
        config_.predictor_sigma_grow_m_sqrt_s;
    const iap::PredictorSpatialSourceUsage source_projection =
        iap::predictorSpatialSourceUsage(predictor_params);
    validate_gnss_spatial_source =
        source_projection.gnss && snapshot.has_epoch;
    validate_lidar_spatial_source = source_projection.lidar;
    validate_lidar_legacy_source = source_projection.legacy_lidar;
    active_gnss_occupancy_content =
        source_projection.gnss && snapshot.has_epoch;
    if (active_gnss_occupancy_content) {
      if (!occupancy_epoch->raw_identity) {
        fail_semantic_refresh(
            P0SemanticFailure::OCCUPANCY_LOS_ADAPTER_INVALID);
        return;
      }
      const bool has_committed_base =
          rolling_occupancy_owner_ && rolling_raw_occupancy_identity_ &&
          rolling_occupancy_source_owner_ &&
          rolling_occupancy_geometry_.valid() &&
          rolling_occupancy_generation_ != 0u &&
          std::isfinite(rolling_occupancy_stamp_) &&
          rolling_occupancy_content_identity_ != 0u;
      const bool partial_committed_base =
          static_cast<bool>(rolling_occupancy_owner_) ||
          static_cast<bool>(rolling_raw_occupancy_identity_) ||
          static_cast<bool>(rolling_occupancy_source_owner_) ||
          rolling_occupancy_geometry_.valid() ||
          rolling_occupancy_generation_ != 0u ||
          std::isfinite(rolling_occupancy_stamp_) ||
          rolling_occupancy_content_identity_ != 0u;
      if (!has_committed_base && partial_committed_base) {
        fail_semantic_refresh(
            P0SemanticFailure::OCCUPANCY_LOS_ADAPTER_INVALID);
        return;
      }

      bool retain_committed_content = false;
      if (has_committed_base) {
        P0OccupancyEpoch committed_base;
        committed_base.los_owner = rolling_occupancy_owner_;
        committed_base.raw_identity = rolling_raw_occupancy_identity_;
        committed_base.source_owner = rolling_occupancy_source_owner_;
        committed_base.geometry = rolling_occupancy_geometry_;
        committed_base.generation = rolling_occupancy_generation_;
        committed_base.cloud_stamp_s = rolling_occupancy_stamp_;
        committed_base.frame_id = rolling_raw_occupancy_identity_->frameId();
        if (sameSharedOwner(rolling_occupancy_source_owner_,
                            occupancy_epoch->source_owner)) {
          if (occupancy_epoch->generation == rolling_occupancy_generation_) {
            if (!P0OccupancyEpochAdapter::sameVersion(
                    committed_base, *occupancy_epoch)) {
              fail_semantic_refresh(
                  P0SemanticFailure::OCCUPANCY_LOS_ADAPTER_INVALID);
              return;
            }
            retain_committed_content = true;
          } else if (occupancy_epoch->generation <
                     rolling_occupancy_generation_) {
            fail_semantic_refresh(
                P0SemanticFailure::OCCUPANCY_LOS_ADAPTER_INVALID);
            return;
          } else {
            const auto delta = P0OccupancyEpochAdapter::completeDelta(
                committed_base, *occupancy_epoch);
            retain_committed_content = delta && delta->empty();
          }
        }
      }

      if (retain_committed_content) {
        occupancy_epoch->los_owner = rolling_occupancy_owner_;
        candidate_occupancy_content_identity =
            rolling_occupancy_content_identity_;
      } else {
        if (rolling_occupancy_content_identity_ ==
            std::numeric_limits<uint64_t>::max()) {
          fail_semantic_refresh(
              P0SemanticFailure::OCCUPANCY_LOS_ADAPTER_INVALID);
          return;
        }
        candidate_occupancy_content_identity =
            rolling_occupancy_content_identity_ + 1u;
      }
    }
    if (std::isfinite(config_.predictor_lidar_fim_radius_m) &&
        config_.predictor_lidar_fim_radius_m > 0.0) {
      predictor_params.lidar.fim_params.fim_radius_m =
          config_.predictor_lidar_fim_radius_m;
      predictor_params.lidar.fim_params.search_radius_m =
          config_.predictor_lidar_fim_radius_m;
    }
    std::shared_ptr<const std::vector<Eigen::Vector3d>> lidar_map_points;
    std::shared_ptr<const std::vector<iap::LidarFimPrimitive>>
        lidar_fim_primitives;
    {
      std::lock_guard<std::mutex> lock(lidar_predictor_input_mutex_);
      lidar_map_points = latest_lidar_map_points_;
      lidar_fim_primitives = latest_lidar_fim_primitives_;
      captured_lidar_generation = latest_lidar_generation_;
      captured_lidar_stamp = latest_lidar_stamp_;
    }
    captured_lidar_map_points = lidar_map_points;
    captured_lidar_fim_primitives = lidar_fim_primitives;
    iap::PredictorModule module(predictor_params);
    if (config_.online_mapping_mode) {
      const auto observed_support_query = occupancy_epoch->diagnostic_query;
      module.set_observation_predicate(
          [observed_support_query](const Eigen::Vector3d& position) {
            const auto diagnostic = observed_support_query(position);
            return diagnostic.available && diagnostic.observed &&
                   diagnostic.state != iap::RiskOccupancyState::UNKNOWN;
          });
    }
    module.set_lidar_map_points(lidar_map_points);
    module.set_lidar_fim_primitives(lidar_fim_primitives);
    forward_risk_module =
        std::make_shared<iap::PredictorModule>(predictor_params);
    forward_risk_occupancy_owner = occupancy_epoch->los_owner;
    forward_risk_module->set_local_occupancy(
        forward_risk_occupancy_owner.get());
    if (config_.online_mapping_mode) {
      const auto observed_support_query = occupancy_epoch->diagnostic_query;
      forward_risk_module->set_observation_predicate(
          [observed_support_query](const Eigen::Vector3d& position) {
            const auto diagnostic = observed_support_query(position);
            return diagnostic.available && diagnostic.observed &&
                   diagnostic.state != iap::RiskOccupancyState::UNKNOWN;
          });
    }
    forward_risk_module->set_lidar_map_points(lidar_map_points);
    forward_risk_module->set_lidar_fim_primitives(lidar_fim_primitives);
    iap::RollingSpatialWindowGeometry rolling_geometry;
    rolling_geometry.frame_id = config_.grid.frame_id;
    rolling_geometry.lattice_anchor_w = config_.grid.lattice_anchor_w;
    rolling_geometry.resolution_m = config_.grid.resolution_m;
    rolling_geometry.shape = risk_grid_.voxelNum();
    iap::RollingSpatialRetentionPolicy retention_policy;
    retention_policy.gnss_spatial_ttl_s =
        config_.predictor_gnss_spatial_ttl_s;
    retention_policy.legacy_current_spatial_ttl_s =
        config_.predictor_legacy_current_spatial_ttl_s;
    retention_policy.full_refresh_watchdog_s =
        config_.predictor_full_refresh_watchdog_s;
    iap::RollingSpatialSourceProvenance source_provenance;
    source_provenance.gnss_epoch_generation =
        captured_predictor_sources.gnss_epoch_generation;
    source_provenance.gnss_epoch_stamp =
        captured_predictor_sources.gnss_epoch_stamp;
    source_provenance.occupancy_generation = occupancy_epoch->generation;
    source_provenance.occupancy_stamp = occupancy_epoch->cloud_stamp_s;
    source_provenance.occupancy_content_identity =
        candidate_occupancy_content_identity;
    source_provenance.lidar_generation = captured_lidar_generation;
    source_provenance.lidar_stamp = captured_lidar_stamp;
    source_provenance.current_generation =
        captured_predictor_sources.current_generation;
    source_provenance.current_stamp =
        captured_predictor_sources.current_stamp;
    source_provenance.refresh_reference_time_s = now_s;
    auto owned_predictor_provider = std::make_unique<PredictorModuleRiskProvider>(
        &rolling_spatial_window_, std::move(rolling_geometry),
        occupancy_epoch->los_owner,
        std::move(lidar_map_points), std::move(lidar_fim_primitives),
        retention_policy, source_provenance,
        std::move(module), snapshot,
        config_.predictor_effective_worker_count,
        config_.predictor_hal_m, config_.predictor_val_m);
    predictor_provider = owned_predictor_provider.get();
    local_provider = std::move(owned_predictor_provider);
    provider = local_provider.get();
  }

  if (predictor_provider && !predictor_provider->ready()) {
    const std::string failure_reason = predictor_provider->beginFailureReason();
    risk_grid_.markRefreshFailure(now_s, failure_reason);
    const double refresh_end_stamp_s = liveNowSeconds();
    {
      std::lock_guard<std::mutex> health_lock(health_state_mutex_);
      last_snapshot_failure_reason_ = failure_reason;
      last_rolling_spatial_diagnostics_ =
          predictor_provider->rollingDiagnostics();
      last_refresh_elapsed_ms_ = std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - refresh_start).count();
      last_refresh_end_stamp_s_ = refresh_end_stamp_s;
      last_refresh_end_steady_s_ = steadyNowSeconds();
    }
    const iap::RiskGridHealth failed_health = risk_grid_.health(now_s);
    completeRefreshEvidence(failed_health, now_s, false);
    publishHealth(failed_health, now_s);
    return;
  }

  std::string reason;
  const Eigen::Vector3i voxel_num = risk_grid_.voxelNum();
  const auto layer_voxel_count =
      static_cast<std::size_t>(std::max(0, voxel_num.x())) *
      static_cast<std::size_t>(std::max(0, voxel_num.y())) *
      static_cast<std::size_t>(std::max(0, voxel_num.z()));
  {
    std::lock_guard<std::mutex> health_lock(health_state_mutex_);
    last_refresh_query_count_ =
        layer_voxel_count * config_.grid.horizons_s.size();
  }
  const auto combined_occupancy_diagnostic_query =
      combinedOccupancyDiagnosticQuery(
      occupancy_epoch ? occupancy_epoch->diagnostic_query
                      : iap::RiskGridMap::OccupancyDiagnosticQuery{});
  const iap::RiskGridMap::OccupancyDiagnosticQuery occupancy_diagnostic_query =
      combined_occupancy_diagnostic_query;
  const auto occupancy_predicate = combinedOccupancyPredicate();
  iap::RiskGridMap::SourceValidator source_validator;
  if (occupancy_epoch) {
    const auto captured_occupancy_source_owner =
        occupancy_epoch->source_owner;
    const auto live_occupancy_source_owner =
        occupancy_epoch->live_source_owner;
    const auto live_occupancy_generation = occupancy_epoch->live_generation;
    const uint64_t captured_occupancy_generation =
        occupancy_epoch->generation;
    const uint64_t captured_prior_generation =
        snapshot.prior_source_generation;
    source_validator =
        [this, captured_occupancy_source_owner,
         live_occupancy_source_owner, live_occupancy_generation,
         captured_occupancy_generation,
         captured_prior_generation, validate_gnss_spatial_source,
         validate_lidar_spatial_source, validate_lidar_legacy_source,
         captured_predictor_sources, captured_lidar_generation,
         captured_lidar_stamp, captured_lidar_map_points,
         captured_lidar_fim_primitives]() {
          if (!captured_occupancy_source_owner ||
              !live_occupancy_source_owner || !live_occupancy_generation ||
              captured_occupancy_generation == 0u) {
            return iap::RiskGridSourceValidation::
                OCCUPANCY_GENERATION_CHANGED;
          }
          const uint64_t live_occupancy_version =
              live_occupancy_generation();
          if (live_occupancy_version == 0u ||
              live_occupancy_version < captured_occupancy_generation ||
              (live_occupancy_version == captured_occupancy_generation &&
               !sameSharedOwner(captured_occupancy_source_owner,
                                live_occupancy_source_owner()))) {
            return iap::RiskGridSourceValidation::
                OCCUPANCY_GENERATION_CHANGED;
          }
          uint64_t live_prior_generation = 0;
          uint64_t live_gnss_epoch_generation = 0;
          bool live_gnss_epoch_available = false;
          double live_current_stamp =
              std::numeric_limits<double>::quiet_NaN();
          double live_gnss_epoch_stamp =
              std::numeric_limits<double>::quiet_NaN();
          {
            std::lock_guard<std::mutex> lock(health_state_mutex_);
            live_prior_generation = latest_current_generation_;
            live_gnss_epoch_generation = latest_gnss_epoch_generation_;
            live_gnss_epoch_available = latest_epoch_.has_value();
            live_current_stamp = latest_current_.stamp;
            live_gnss_epoch_stamp = latest_epoch_
                ? latest_epoch_->stamp
                : std::numeric_limits<double>::quiet_NaN();
          }
          if (captured_prior_generation == 0u ||
              captured_predictor_sources.current_generation == 0u ||
              captured_prior_generation !=
                  captured_predictor_sources.current_generation ||
              !std::isfinite(captured_predictor_sources.current_stamp) ||
              live_prior_generation < captured_prior_generation ||
              (live_prior_generation == captured_prior_generation &&
               live_current_stamp !=
                   captured_predictor_sources.current_stamp)) {
            return iap::RiskGridSourceValidation::PRIOR_GENERATION_CHANGED;
          }
          if (validate_gnss_spatial_source) {
            if (captured_predictor_sources.gnss_epoch_generation == 0u ||
                !std::isfinite(
                    captured_predictor_sources.gnss_epoch_stamp) ||
                live_gnss_epoch_generation <
                    captured_predictor_sources.gnss_epoch_generation ||
                (live_gnss_epoch_generation >
                     captured_predictor_sources.gnss_epoch_generation &&
                 (!live_gnss_epoch_available ||
                  !std::isfinite(live_gnss_epoch_stamp))) ||
                (live_gnss_epoch_generation ==
                     captured_predictor_sources.gnss_epoch_generation &&
                 live_gnss_epoch_stamp !=
                     captured_predictor_sources.gnss_epoch_stamp)) {
              return iap::RiskGridSourceValidation::
                  PREDICTOR_SPATIAL_SOURCE_CHANGED;
            }
          }
          if (validate_lidar_spatial_source) {
            std::lock_guard<std::mutex> lock(lidar_predictor_input_mutex_);
            if (captured_lidar_generation == 0u ||
                !captured_lidar_fim_primitives ||
                latest_lidar_generation_ < captured_lidar_generation ||
                (latest_lidar_generation_ == captured_lidar_generation &&
                 (!sameSharedOwner(latest_lidar_fim_primitives_,
                                   captured_lidar_fim_primitives) ||
                  latest_lidar_stamp_ != captured_lidar_stamp))) {
              return iap::RiskGridSourceValidation::
                  PREDICTOR_SPATIAL_SOURCE_CHANGED;
            }
            if (validate_lidar_legacy_source &&
                (!captured_lidar_map_points ||
                 (latest_lidar_generation_ == captured_lidar_generation &&
                  !sameSharedOwner(latest_lidar_map_points_,
                                   captured_lidar_map_points)))) {
              return iap::RiskGridSourceValidation::
                  PREDICTOR_SPATIAL_SOURCE_CHANGED;
            }
          }
          return iap::RiskGridSourceValidation::VALID;
        };
  }
  TimedRiskProvider timed_provider(provider);
  iap::RiskGridSourceIdentity source_identity;
  source_identity.occupancy_generation = occupancy_epoch
      ? occupancy_epoch->generation : 0u;
  source_identity.occupancy_stamp_s = occupancy_epoch
      ? occupancy_epoch->cloud_stamp_s
      : std::numeric_limits<double>::quiet_NaN();
  source_identity.prior_generation =
      captured_predictor_sources.current_generation;
  source_identity.prior_stamp_s = captured_predictor_sources.current_stamp;
  source_identity.gnss_generation =
      captured_predictor_sources.gnss_epoch_generation;
  source_identity.gnss_stamp_s =
      captured_predictor_sources.gnss_epoch_stamp;
  source_identity.lidar_generation = captured_lidar_generation;
  source_identity.lidar_stamp_s = captured_lidar_stamp;
  source_identity.alert_limit_policy_id =
      config_.grid.alert_limit_policy_id;
  bool refresh_succeeded = false;
  if (occupancy_diagnostic_query) {
    refresh_succeeded = risk_grid_.refreshFromProvider(
        snapshot.p_wb, now_s, timed_provider,
        occupancy_diagnostic_query, source_validator, source_identity,
        &reason);
  } else {
    refresh_succeeded = risk_grid_.refreshFromProvider(
        snapshot.p_wb, now_s, timed_provider, occupancy_predicate, &reason);
  }
  const auto viz_snapshot = risk_grid_.acquireSnapshot();
  if (refresh_succeeded && viz_snapshot) {
    const auto& identity = viz_snapshot->sourceIdentity();
    const bool occupancy_identity_matches = !occupancy_epoch ||
        (identity.occupancy_generation == occupancy_epoch->generation &&
         identity.occupancy_stamp_s == occupancy_epoch->cloud_stamp_s);
    if (occupancy_identity_matches) {
      auto planning = std::make_shared<P0PlanningSnapshot>();
      planning->risk = viz_snapshot;
      planning->integrity_anchor = snapshot;
      planning->gnss_hard_occlusion = forward_gnss_hard_occlusion;
      planning->gnss_support_ray_length_m =
          forward_gnss_support_ray_length_m;
      if (occupancy_epoch) {
        planning->occupancy =
            std::make_shared<P0OccupancyEpoch>(*occupancy_epoch);
      }
      if (forward_risk_module && forward_risk_occupancy_owner) {
        planning->forward_risk_batch =
            [forward_risk_module, forward_risk_occupancy_owner, snapshot,
             hal = config_.predictor_hal_m,
             val = config_.predictor_val_m](
                const iap::ForwardRiskBatchRequest& input) {
              (void)forward_risk_occupancy_owner;
              iap::ForwardRiskBatchRequest request = input;
              request.snapshot = snapshot;
              request.hal = hal;
              request.val = val;
              return forward_risk_module->queryForwardRiskBatch(request);
            };
      }
      std::lock_guard<std::mutex> lock(planning_snapshot_mutex_);
      planning_snapshot_ = std::move(planning);
    } else {
      refresh_succeeded = false;
      reason = "risk_occupancy_commit_identity_mismatch";
      risk_grid_.markRefreshFailure(now_s, reason);
    }
  }
  const iap::RiskGridHealth health = risk_grid_.health(now_s);
  const double refresh_end_stamp_s = liveNowSeconds();
  iap::RollingSpatialRefreshDiagnostics rolling_diagnostics;
  if (predictor_provider) {
    rolling_diagnostics = predictor_provider->rollingDiagnostics();
    predictor_provider->finish(refresh_succeeded);
    if (!refresh_succeeded) {
      rolling_diagnostics = {};
    }
    if (refresh_succeeded && occupancy_epoch &&
        active_gnss_occupancy_content) {
      rolling_occupancy_owner_ = occupancy_epoch->los_owner;
      rolling_raw_occupancy_identity_ = occupancy_epoch->raw_identity;
      rolling_occupancy_source_owner_ = occupancy_epoch->source_owner;
      rolling_occupancy_geometry_ = occupancy_epoch->geometry;
      rolling_occupancy_generation_ = occupancy_epoch->generation;
      rolling_occupancy_stamp_ = occupancy_epoch->cloud_stamp_s;
      rolling_occupancy_content_identity_ =
          candidate_occupancy_content_identity;
    }
  }
  {
    std::lock_guard<std::mutex> health_lock(health_state_mutex_);
    last_grid_stamp_s_ = viz_snapshot ? viz_snapshot->stamp_s()
                                      : std::numeric_limits<double>::quiet_NaN();
    last_refresh_elapsed_ms_ = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - refresh_start).count();
    last_provider_batch_duration_ms_ = timed_provider.durationMs();
    if (predictor_provider)
    {
      const auto& diagnostics = predictor_provider->diagnostics();
      last_predictor_unique_positions_ = diagnostics.unique_positions;
      last_predictor_lidar_evaluations_ = diagnostics.lidar_evaluations;
      last_predictor_lidar_cache_hits_ = diagnostics.lidar_cache_hits;
      last_predictor_spatial_advisory_recompute_count_ =
          diagnostics.spatial_advisory_recompute_count;
      last_predictor_spatial_advisory_reuse_count_ =
          diagnostics.spatial_advisory_reuse_count;
      last_predictor_gnss_advisory_invocation_count_ =
          diagnostics.gnss_advisory_invocations;
      last_predictor_lidar_advisory_invocation_count_ =
          diagnostics.lidar_advisory_invocations;
      last_predictor_horizon_fusion_count_ =
          diagnostics.fusion_advisory_invocations;
      last_rolling_spatial_diagnostics_ = rolling_diagnostics;
    }
    last_refresh_end_stamp_s_ = refresh_end_stamp_s;
    last_refresh_succeeded_ = refresh_succeeded;
    last_refresh_end_steady_s_ = steadyNowSeconds();
    if (refresh_succeeded) {
      last_generation_interval_ms_ =
          std::isfinite(last_generation_end_steady_s_)
              ? 1000.0 * (last_refresh_end_steady_s_ -
                          last_generation_end_steady_s_)
              : std::numeric_limits<double>::quiet_NaN();
      last_generation_end_steady_s_ = last_refresh_end_steady_s_;
    } else {
      last_generation_interval_ms_ =
          std::numeric_limits<double>::quiet_NaN();
    }
  }
  completeRefreshEvidence(health, now_s, refresh_succeeded);
  publishHealth(health, now_s);
  if (safety_viz_) {
    safety_viz_->publishPredictedPLCloud(viz_snapshot, snapshot.p_wb.z(),
                                         now_s);
    safety_viz_->publishRiskValidityCloud(viz_snapshot, snapshot.p_wb.z(),
                                          now_s);
  }
}

void P0RiskGridRuntime::publishHealth(const iap::RiskGridHealth& health,
                                      const double now_s) {
  // Never serialize or invoke ROS publishers while the input/refresh state
  // mutex is held. A slow subscriber or RViz transport must not block input
  // callbacks or turn health into a self-fulfilling stale signal.
  const iap::RiskGridHealth enriched_health = addLidarPredictorInputHealth(health);
  const auto mutex_wait_start = std::chrono::steady_clock::now();
  HealthPublicationState state;
  RefreshEvidenceRecord evidence;
  std::string snapshot_failure_reason = "none";
  {
    std::lock_guard<std::mutex> health_lock(health_state_mutex_);
    const auto mutex_acquired = std::chrono::steady_clock::now();
    last_publish_stamp_s_ = now_s;
    last_publish_steady_s_ = steadyNowSeconds();
    const double process_cpu_ms = 1000.0 * static_cast<double>(std::clock()) /
        static_cast<double>(CLOCKS_PER_SEC);
    last_process_cpu_delta_ms_ = std::isfinite(last_process_cpu_ms_)
        ? process_cpu_ms - last_process_cpu_ms_
        : std::numeric_limits<double>::quiet_NaN();
    last_process_cpu_ms_ = process_cpu_ms;
    const HealthPublicationState live_state = healthPublicationStateSnapshot();
    evidence = refresh_evidence_;
    state = evidence.publication;
    state.health_callback_stamp_s = live_state.health_callback_stamp_s;
    state.publish_stamp_s = live_state.publish_stamp_s;
    state.health_callback_steady_s = live_state.health_callback_steady_s;
    state.publish_steady_s = live_state.publish_steady_s;
    state.input_callback_count = live_state.input_callback_count;
    state.health_callback_count = live_state.health_callback_count;
    state.process_cpu_delta_ms = live_state.process_cpu_delta_ms;
    state.health_callback_duration_ms = live_state.health_callback_duration_ms;
    state.health_callback_queue_delay_ms = live_state.health_callback_queue_delay_ms;
    state.input_callback_age_s =
        std::isfinite(last_input_callback_steady_s_)
            ? std::max(0.0, last_publish_steady_s_ -
                                last_input_callback_steady_s_)
            : std::numeric_limits<double>::quiet_NaN();
    last_health_state_mutex_wait_ms_ =
        std::chrono::duration<double, std::milli>(mutex_acquired - mutex_wait_start).count();
    last_health_state_mutex_hold_ms_ =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - mutex_acquired).count();
    state.health_state_mutex_wait_ms = last_health_state_mutex_wait_ms_;
    state.health_state_mutex_hold_ms = last_health_state_mutex_hold_ms_;
    snapshot_failure_reason = evidence.snapshot_failure_reason;
  }
  if (safety_viz_) {
    safety_viz_->publishRiskGridHealth(enriched_health, now_s);
  }
  if (!node_ || !health_pub_) {
    return;
  }
  const iap::RiskGridHealth& out_health = evidence.health;
  const InputReadiness& readiness = evidence.readiness;
  const auto& health_snapshot = evidence.snapshot;
  const iap::RiskGridMapParams* grid_params = health_snapshot
      ? &health_snapshot->params() : nullptr;
  const Eigen::Vector3d grid_origin = health_snapshot
      ? health_snapshot->origin()
      : Eigen::Vector3d::Constant(
          std::numeric_limits<double>::quiet_NaN());
  const Eigen::Vector3i grid_dimensions = health_snapshot
      ? health_snapshot->voxelNum() : Eigen::Vector3i::Zero();
  const iap::RiskGridSourceIdentity source_identity = health_snapshot
      ? health_snapshot->sourceIdentity() : iap::RiskGridSourceIdentity{};
  const std::string snapshot_config_hash = health_snapshot
      ? iap::canonicalRiskGridConfigHash(health_snapshot->params()) : "";
  const auto planning_snapshot = acquirePlanningSnapshot();
  const bool anchor_matches_health = planning_snapshot && health_snapshot &&
      planning_snapshot->risk.get() == health_snapshot.get();
  const auto& integrity_anchor = anchor_matches_health
      ? planning_snapshot->integrity_anchor.current
      : iap::CurrentIntegrityState{};
  std::ostringstream oss;
  oss << "{"
      << "\"refresh_attempt_id\":" << evidence.refresh_attempt_id << ","
      << "\"refresh_evidence_state\":"
      << jsonString(refreshEvidenceStateName(evidence.state)) << ","
      << "\"result_generation_id\":"
      << evidence.result_generation_id << ","
      << "\"previous_successful_generation_id\":"
      << evidence.previous_successful_generation_id << ","
      << "\"ready\":" << (out_health.ready ? "true" : "false") << ","
      << "\"stale\":" << (out_health.stale ? "true" : "false") << ","
      << "\"age_s\":" << jsonNumber(out_health.age_s) << ","
      << "\"valid_ratio\":" << jsonNumber(out_health.valid_ratio) << ","
      << "\"unknown_ratio\":" << jsonNumber(out_health.unknown_ratio) << ","
      << "\"generation_id\":" << out_health.generation_id << ","
      << "\"snapshot_config_hash\":"
      << jsonString(snapshot_config_hash) << ","
      << "\"source_identity_hash\":"
      << jsonString(iap::canonicalRiskGridSourceIdentityHash(
             source_identity)) << ","
      << "\"snapshot_identity_available\":"
      << (health_snapshot ? "true" : "false") << ","
      << "\"geometry_id\":"
      << jsonString(grid_params ? grid_params->geometry_id : "") << ","
      << "\"frame_id\":"
      << jsonString(grid_params ? grid_params->frame_id : "") << ","
      << "\"grid_origin_m\":[" << jsonNumber(grid_origin.x()) << ","
      << jsonNumber(grid_origin.y()) << "," << jsonNumber(grid_origin.z())
      << "],"
      << "\"grid_extent_m\":["
      << jsonNumber(grid_params ? grid_params->size_x_m
                                : std::numeric_limits<double>::quiet_NaN())
      << ","
      << jsonNumber(grid_params ? grid_params->size_y_m
                                : std::numeric_limits<double>::quiet_NaN())
      << ","
      << jsonNumber(grid_params ? grid_params->size_z_m
                                : std::numeric_limits<double>::quiet_NaN())
      << "],"
      << "\"grid_dimensions\":[" << grid_dimensions.x() << ","
      << grid_dimensions.y() << "," << grid_dimensions.z() << "],"
      << "\"grid_resolution_m\":"
      << jsonNumber(grid_params ? grid_params->resolution_m
                                : std::numeric_limits<double>::quiet_NaN())
      << ","
      << "\"alert_limit_policy_id\":"
      << jsonString(grid_params ? grid_params->alert_limit_policy_id : "")
      << ","
      << "\"alert_limit_h_m\":"
      << jsonNumber(grid_params ? grid_params->alert_limit_h_m
                                : std::numeric_limits<double>::quiet_NaN())
      << ",\"alert_limit_v_m\":"
      << jsonNumber(grid_params ? grid_params->alert_limit_v_m
                                : std::numeric_limits<double>::quiet_NaN())
      << ","
      << "\"gnss_anchor_available\":"
      << (anchor_matches_health && integrity_anchor.valid &&
                  integrity_anchor.gnss_valid
              ? "true" : "false") << ","
      << "\"gnss_anchor_hpl_m\":"
      << jsonNumber(anchor_matches_health ? integrity_anchor.hpl
                                          : std::numeric_limits<double>::quiet_NaN())
      << ",\"gnss_anchor_vpl_m\":"
      << jsonNumber(anchor_matches_health ? integrity_anchor.vpl
                                          : std::numeric_limits<double>::quiet_NaN())
      << ",\"gnss_anchor_stamp_s\":"
      << jsonNumber(anchor_matches_health ? integrity_anchor.stamp
                                          : std::numeric_limits<double>::quiet_NaN())
      << ",\"gnss_anchor_epoch_delta_s\":"
      << jsonNumber(anchor_matches_health
            ? std::abs(integrity_anchor.stamp -
                       planning_snapshot->integrity_anchor.gnss_epoch.stamp)
            : std::numeric_limits<double>::quiet_NaN())
      << ",\"gnss_support_ray_length_m\":"
      << jsonNumber(anchor_matches_health
            ? planning_snapshot->gnss_support_ray_length_m
            : std::numeric_limits<double>::quiet_NaN())
      << ",\"gnss_hard_occlusion\":"
      << (anchor_matches_health && planning_snapshot->gnss_hard_occlusion
              ? "true" : "false") << ","
      << "\"source_occupancy_generation\":"
      << source_identity.occupancy_generation << ","
      << "\"source_occupancy_stamp_s\":"
      << jsonNumber(source_identity.occupancy_stamp_s) << ","
      << "\"source_prior_generation\":"
      << source_identity.prior_generation << ","
      << "\"source_prior_stamp_s\":"
      << jsonNumber(source_identity.prior_stamp_s) << ","
      << "\"source_gnss_generation\":"
      << source_identity.gnss_generation << ","
      << "\"source_gnss_stamp_s\":"
      << jsonNumber(source_identity.gnss_stamp_s) << ","
      << "\"source_lidar_generation\":"
      << source_identity.lidar_generation << ","
      << "\"source_lidar_stamp_s\":"
      << jsonNumber(source_identity.lidar_stamp_s) << ","
      << "\"provider_query_count\":" << out_health.provider_query_count << ","
      << "\"occupied_skip_count\":" << out_health.occupied_skip_count << ","
      << "\"provider_stale_count\":" << out_health.provider_stale_count << ","
      << "\"provider_invalid_count\":" << out_health.provider_invalid_count << ","
      << "\"predictor_gnss_used_count\":"
      << out_health.predictor_gnss_used_count << ","
      << "\"predictor_lidar_used_count\":"
      << out_health.predictor_lidar_used_count << ","
      << "\"predictor_prior_used_count\":"
      << out_health.predictor_prior_used_count << ","
      << "\"predictor_stale_current_prior_count\":"
      << out_health.predictor_stale_current_prior_count << ","
      << "\"predictor_regularized_count\":"
      << out_health.predictor_regularized_count << ","
      << "\"predictor_conservative_max_count\":"
      << out_health.predictor_conservative_max_count << ","
      << "\"predictor_lidar_map_point_count\":"
      << out_health.predictor_lidar_map_point_count << ","
      << "\"predictor_lidar_fim_primitive_count\":"
      << out_health.predictor_lidar_fim_primitive_count << ","
      << "\"predictor_lidar_fim_valid_normal_count\":"
      << out_health.predictor_lidar_fim_valid_normal_count << ","
      << "\"predictor_lidar_fim_fallback_reason\":"
      << jsonString(out_health.predictor_lidar_fim_fallback_reason) << ","
      << "\"dominant_unknown_reason\":"
      << jsonString(out_health.dominant_unknown_reason) << ","
      << "\"dominant_unknown_count\":"
      << out_health.dominant_unknown_count << ","
      << "\"refresh_stamp_s\":" << jsonNumber(state.refresh_stamp_s) << ","
      << "\"refresh_callback_start_stamp_s\":" << jsonNumber(state.refresh_start_stamp_s) << ","
      << "\"refresh_callback_end_stamp_s\":" << jsonNumber(state.refresh_end_stamp_s) << ","
      << "\"health_callback_stamp_s\":" << jsonNumber(state.health_callback_stamp_s) << ","
      << "\"publish_stamp_s\":" << jsonNumber(state.publish_stamp_s) << ","
      << "\"refresh_callback_start_steady_s\":" << jsonNumber(state.refresh_start_steady_s) << ","
      << "\"refresh_scheduled_steady_s\":" << jsonNumber(state.refresh_scheduled_steady_s) << ","
      << "\"refresh_callback_end_steady_s\":" << jsonNumber(state.refresh_end_steady_s) << ","
      << "\"health_callback_steady_s\":" << jsonNumber(state.health_callback_steady_s) << ","
      << "\"publish_steady_s\":" << jsonNumber(state.publish_steady_s) << ","
      << "\"last_grid_stamp_s\":" << jsonNumber(state.last_grid_stamp_s) << ","
      << "\"refresh_elapsed_ms\":" << jsonNumber(state.refresh_elapsed_ms) << ","
      << "\"refresh_duration_ms\":" << jsonNumber(state.refresh_elapsed_ms) << ","
      << "\"refresh_queue_delay_ms\":" << jsonNumber(state.refresh_queue_delay_ms) << ","
      << "\"provider_batch_duration_ms\":" << jsonNumber(state.provider_batch_duration_ms) << ","
      << "\"generation_interval_ms\":" << jsonNumber(state.generation_interval_ms) << ","
      << "\"input_callback_age_s\":" << jsonNumber(state.input_callback_age_s) << ","
      << "\"input_callback_count\":" << state.input_callback_count << ","
      << "\"health_callback_count\":" << state.health_callback_count << ","
      << "\"process_cpu_delta_ms\":" << jsonNumber(state.process_cpu_delta_ms) << ","
      << "\"health_callback_duration_ms\":" << jsonNumber(state.health_callback_duration_ms) << ","
      << "\"health_callback_queue_delay_ms\":" << jsonNumber(state.health_callback_queue_delay_ms) << ","
      << "\"health_state_mutex_wait_ms\":" << jsonNumber(state.health_state_mutex_wait_ms) << ","
      << "\"health_state_mutex_hold_ms\":" << jsonNumber(state.health_state_mutex_hold_ms) << ","
      << "\"snapshot_available\":"
      << (state.snapshot_available ? "true" : "false") << ","
      << "\"snapshot_failure_reason\":"
      << jsonString(snapshot_failure_reason) << ","
      << "\"odom_seen\":" << jsonBool(readiness.odom_seen) << ","
      << "\"odom_valid\":" << jsonBool(readiness.odom_valid) << ","
      << "\"odom_fresh\":" << jsonBool(readiness.odom_fresh) << ","
      << "\"odom_stamp_s\":" << jsonNumber(readiness.odom_stamp_s) << ","
      << "\"current_integrity_seen\":"
      << jsonBool(readiness.current_integrity_seen) << ","
      << "\"current_integrity_valid\":"
      << jsonBool(readiness.current_integrity_valid) << ","
      << "\"current_integrity_fresh\":"
      << jsonBool(readiness.current_integrity_fresh) << ","
      << "\"current_integrity_stamp_s\":"
      << jsonNumber(readiness.current_integrity_stamp_s) << ","
      << "\"gnss_epoch_seen\":" << jsonBool(readiness.gnss_epoch_seen) << ","
      << "\"gnss_epoch_valid\":" << jsonBool(readiness.gnss_epoch_valid) << ","
      << "\"gnss_epoch_fresh\":" << jsonBool(readiness.gnss_epoch_fresh) << ","
      << "\"gnss_epoch_stamp_s\":"
      << jsonNumber(readiness.gnss_epoch_stamp_s) << ","
      << "\"gnss_epoch_satellite_count\":"
      << readiness.gnss_epoch_satellite_count << ","
      << "\"origin_seen\":" << jsonBool(readiness.origin_seen) << ","
      << "\"origin_valid\":" << jsonBool(readiness.origin_valid) << ","
      << "\"origin_fresh\":" << jsonBool(readiness.origin_fresh) << ","
      << "\"origin_stamp_s\":" << jsonNumber(readiness.origin_stamp_s) << ","
      << "\"map_seen\":" << jsonBool(readiness.map_seen) << ","
      << "\"map_valid\":" << jsonBool(readiness.map_valid) << ","
      << "\"map_fresh\":" << jsonBool(readiness.map_fresh) << ","
      << "\"map_stamp_s\":" << jsonNumber(readiness.map_stamp_s) << ","
      << "\"map_point_count\":" << readiness.map_point_count << ","
      << "\"refresh_query_count\":" << state.refresh_query_count << ","
      << "\"predictor_unique_positions\":" << state.predictor_unique_positions << ","
      << "\"predictor_lidar_evaluations\":" << state.predictor_lidar_evaluations << ","
      << "\"predictor_lidar_cache_hits\":" << state.predictor_lidar_cache_hits << ","
      << "\"predictor_spatial_advisory_recompute_count\":"
      << state.predictor_spatial_advisory_recompute_count << ","
      << "\"predictor_spatial_advisory_reuse_count\":"
      << state.predictor_spatial_advisory_reuse_count << ","
      << "\"predictor_gnss_advisory_invocation_count\":"
      << state.predictor_gnss_advisory_invocation_count << ","
      << "\"predictor_lidar_advisory_invocation_count\":"
      << state.predictor_lidar_advisory_invocation_count << ","
      << "\"predictor_horizon_fusion_count\":"
      << state.predictor_horizon_fusion_count << ","
      << "\"predictor_spatial_retained_position_count\":"
      << state.predictor_spatial_retained_position_count << ","
      << "\"predictor_spatial_entered_position_count\":"
      << state.predictor_spatial_entered_position_count << ","
      << "\"predictor_spatial_evicted_position_count\":"
      << state.predictor_spatial_evicted_position_count << ","
      << "\"predictor_spatial_full_invalidation_count\":"
      << state.predictor_spatial_full_invalidation_count << ","
      << "\"predictor_spatial_exact_retained_position_count\":"
      << state.predictor_spatial_exact_retained_position_count << ","
      << "\"predictor_spatial_ttl_retained_position_count\":"
      << state.predictor_spatial_ttl_retained_position_count << ","
      << "\"predictor_spatial_gnss_ttl_expired_position_count\":"
      << state.predictor_spatial_gnss_ttl_expired_position_count << ","
      << "\"predictor_spatial_legacy_current_ttl_expired_position_count\":"
      << state.predictor_spatial_legacy_current_ttl_expired_position_count
      << ","
      << "\"predictor_spatial_watchdog_forced_full_rebuild_count\":"
      << state.predictor_spatial_watchdog_forced_full_rebuild_count << ","
      << "\"predictor_spatial_invalid_source_provenance_count\":"
      << state.predictor_spatial_invalid_source_provenance_count << ","
      << "\"predictor_spatial_invalidation_reason\":"
      << jsonString(state.predictor_spatial_invalidation_reason) << ","
      << "\"predictor_requested_worker_count\":" << config_.predictor_requested_worker_count << ","
      << "\"predictor_effective_worker_count\":" << config_.predictor_effective_worker_count << ","
      << "\"reason\":" << jsonString(out_health.reason)
      << "}";
  std_msgs::msg::String msg;
  msg.data = oss.str();
  health_pub_->publish(msg);
  RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                       "[p0] risk grid ready=%d stale=%d age=%.3f "
                       "valid=%.3f unknown=%.3f gen=%lu "
                       "provider_queries=%lu occupied_skip=%lu "
                       "provider_stale=%lu provider_invalid=%lu "
                       "gnss_used=%lu lidar_used=%lu prior_used=%lu "
                       "stale_current_prior=%lu "
                       "regularized=%lu conservative_max=%lu "
                       "lidar_points=%lu lidar_fim_primitives=%lu "
                       "lidar_fim_valid_normals=%lu lidar_fim_fallback=%s "
                       "dominant_unknown=%s:%lu "
                       "refresh_stamp=%.6f grid_stamp=%.6f elapsed_ms=%.3f "
                       "queries=%zu reason=%s",
                       out_health.ready, out_health.stale, out_health.age_s,
                       out_health.valid_ratio, out_health.unknown_ratio,
                       static_cast<unsigned long>(out_health.generation_id),
                       static_cast<unsigned long>(
                           out_health.provider_query_count),
                       static_cast<unsigned long>(
                           out_health.occupied_skip_count),
                       static_cast<unsigned long>(
                           out_health.provider_stale_count),
                       static_cast<unsigned long>(
                           out_health.provider_invalid_count),
                       static_cast<unsigned long>(
                           out_health.predictor_gnss_used_count),
                       static_cast<unsigned long>(
                           out_health.predictor_lidar_used_count),
                       static_cast<unsigned long>(
                           out_health.predictor_prior_used_count),
                       static_cast<unsigned long>(
                           out_health.predictor_stale_current_prior_count),
                       static_cast<unsigned long>(
                           out_health.predictor_regularized_count),
                       static_cast<unsigned long>(
                           out_health.predictor_conservative_max_count),
                       static_cast<unsigned long>(
                           out_health.predictor_lidar_map_point_count),
                       static_cast<unsigned long>(
                           out_health.predictor_lidar_fim_primitive_count),
                       static_cast<unsigned long>(
                           out_health.predictor_lidar_fim_valid_normal_count),
                       out_health.predictor_lidar_fim_fallback_reason.c_str(),
                       out_health.dominant_unknown_reason.c_str(),
                       static_cast<unsigned long>(
                           out_health.dominant_unknown_count),
                       state.refresh_stamp_s, state.last_grid_stamp_s,
                       state.refresh_elapsed_ms, state.refresh_query_count,
                       out_health.reason.c_str());
}

P0RiskGridRuntime::HealthPublicationState
P0RiskGridRuntime::healthPublicationStateSnapshot() const {
  // Caller owns health_state_mutex_. Keeping this as a copy-only helper makes
  // it hard to regress into publishing under the mutex.
  HealthPublicationState state;
  state.refresh_stamp_s = last_refresh_stamp_s_;
  state.refresh_scheduled_steady_s = last_refresh_scheduled_steady_s_;
  state.refresh_start_stamp_s = last_refresh_start_stamp_s_;
  state.refresh_end_stamp_s = last_refresh_end_stamp_s_;
  state.health_callback_stamp_s = last_health_callback_stamp_s_;
  state.publish_stamp_s = last_publish_stamp_s_;
  state.refresh_start_steady_s = last_refresh_start_steady_s_;
  state.refresh_end_steady_s = last_refresh_end_steady_s_;
  state.health_callback_steady_s = last_health_callback_steady_s_;
  state.publish_steady_s = last_publish_steady_s_;
  state.last_grid_stamp_s = last_grid_stamp_s_;
  state.refresh_elapsed_ms = last_refresh_elapsed_ms_;
  state.refresh_queue_delay_ms = last_refresh_queue_delay_ms_;
  state.provider_batch_duration_ms = last_provider_batch_duration_ms_;
  state.generation_interval_ms = last_generation_interval_ms_;
  state.process_cpu_delta_ms = last_process_cpu_delta_ms_;
  state.health_callback_duration_ms = last_health_callback_duration_ms_;
  state.health_callback_queue_delay_ms = last_health_callback_queue_delay_ms_;
  state.health_state_mutex_wait_ms = last_health_state_mutex_wait_ms_;
  state.health_state_mutex_hold_ms = last_health_state_mutex_hold_ms_;
  state.refresh_query_count = last_refresh_query_count_;
  state.predictor_unique_positions = last_predictor_unique_positions_;
  state.predictor_lidar_evaluations = last_predictor_lidar_evaluations_;
  state.predictor_lidar_cache_hits = last_predictor_lidar_cache_hits_;
  state.predictor_spatial_advisory_recompute_count =
      last_predictor_spatial_advisory_recompute_count_;
  state.predictor_spatial_advisory_reuse_count =
      last_predictor_spatial_advisory_reuse_count_;
  state.predictor_gnss_advisory_invocation_count =
      last_predictor_gnss_advisory_invocation_count_;
  state.predictor_lidar_advisory_invocation_count =
      last_predictor_lidar_advisory_invocation_count_;
  state.predictor_horizon_fusion_count =
      last_predictor_horizon_fusion_count_;
  state.predictor_spatial_retained_position_count =
      last_rolling_spatial_diagnostics_.retained_position_count;
  state.predictor_spatial_entered_position_count =
      last_rolling_spatial_diagnostics_.entered_position_count;
  state.predictor_spatial_evicted_position_count =
      last_rolling_spatial_diagnostics_.evicted_position_count;
  state.predictor_spatial_full_invalidation_count =
      last_rolling_spatial_diagnostics_.full_invalidation_count;
  state.predictor_spatial_exact_retained_position_count =
      last_rolling_spatial_diagnostics_.exact_retained_position_count;
  state.predictor_spatial_ttl_retained_position_count =
      last_rolling_spatial_diagnostics_.ttl_retained_position_count;
  state.predictor_spatial_gnss_ttl_expired_position_count =
      last_rolling_spatial_diagnostics_.gnss_ttl_expired_position_count;
  state.predictor_spatial_legacy_current_ttl_expired_position_count =
      last_rolling_spatial_diagnostics_
          .legacy_current_ttl_expired_position_count;
  state.predictor_spatial_watchdog_forced_full_rebuild_count =
      last_rolling_spatial_diagnostics_
          .watchdog_forced_full_rebuild_count;
  state.predictor_spatial_invalid_source_provenance_count =
      last_rolling_spatial_diagnostics_
          .invalid_source_provenance_count;
  state.predictor_spatial_invalidation_reason =
      iap::rollingSpatialInvalidationReasonName(
          last_rolling_spatial_diagnostics_.invalidation_reason);
  state.input_callback_count = input_callback_count_;
  state.health_callback_count = health_callback_count_;
  state.snapshot_available = last_snapshot_available_;
  return state;
}

P0RiskGridRuntime::RefreshEvidenceRecord
P0RiskGridRuntime::refreshEvidenceRecordSnapshot() const {
  std::lock_guard<std::mutex> lock(health_state_mutex_);
  return refresh_evidence_;
}

const char* P0RiskGridRuntime::refreshEvidenceStateName(
    const RefreshEvidenceState state) {
  switch (state) {
    case RefreshEvidenceState::PRE_REFRESH:
      return "PRE_REFRESH";
    case RefreshEvidenceState::IN_PROGRESS:
      return "IN_PROGRESS";
    case RefreshEvidenceState::COMPLETED_SUCCESS:
      return "COMPLETED_SUCCESS";
    case RefreshEvidenceState::COMPLETED_FAILURE:
      return "COMPLETED_FAILURE";
  }
  return "UNKNOWN";
}

void P0RiskGridRuntime::completeRefreshEvidence(
    const iap::RiskGridHealth& health, const double now_s,
    const bool succeeded) {
  (void)now_s;
  const auto completed_snapshot = risk_grid_.acquireSnapshot();
  std::scoped_lock lock(health_state_mutex_, lidar_predictor_input_mutex_);
  const iap::RiskGridHealth completed_health =
      addLidarPredictorInputHealthLocked(health);
  refresh_evidence_.state = succeeded
      ? RefreshEvidenceState::COMPLETED_SUCCESS
      : RefreshEvidenceState::COMPLETED_FAILURE;
  refresh_evidence_.result_generation_id =
      succeeded ? completed_health.generation_id : 0u;
  refresh_evidence_.previous_successful_generation_id =
      last_successful_generation_id_;
  refresh_evidence_.publication = healthPublicationStateSnapshot();
  refresh_evidence_.readiness = refresh_input_readiness_;
  refresh_evidence_.health = completed_health;
  refresh_evidence_.snapshot.reset();
  if (completed_snapshot &&
      completed_snapshot->generation_id() == completed_health.generation_id) {
    refresh_evidence_.snapshot = completed_snapshot;
  }
  refresh_evidence_.snapshot_failure_reason = last_snapshot_failure_reason_;
  if (succeeded) {
    last_successful_generation_id_ = completed_health.generation_id;
  }
}

P0RiskGridRuntime::InputReadiness
P0RiskGridRuntime::inputReadiness(const double now_s) const {
  std::scoped_lock lock(health_state_mutex_, lidar_predictor_input_mutex_);
  return inputReadinessLocked(now_s);
}

P0RiskGridRuntime::InputReadiness
P0RiskGridRuntime::inputReadinessLocked(const double now_s) const {
  InputReadiness readiness;
  const double odom_stamp = latest_odom_stamp_;
  const double current_stamp = latest_current_.stamp;
  const double map_stamp = latest_map_stamp_;
  const double origin_stamp = latest_origin_stamp_;
  const double gnss_epoch_stamp = latest_epoch_
      ? latest_epoch_->stamp : latest_gnss_epoch_stamp_;
  const uint64_t epoch_satellite_count = latest_epoch_
      ? static_cast<uint64_t>(latest_epoch_->sats.size())
      : latest_gnss_epoch_satellite_count_;
  const std::size_t map_point_count = latest_lidar_map_point_count_;

  const double stale_timeout = config_.grid.stale_timeout_s;
  const double gnss_timeout = config_.gnss_epoch_max_age_s;
  const bool finite_now = std::isfinite(now_s);

  readiness.odom_seen = odom_seen_;
  readiness.odom_valid = odom_seen_ && latest_odom_pose_valid_;
  readiness.odom_stamp_s = odom_stamp;
  readiness.odom_fresh =
      readiness.odom_valid && finite_now && std::isfinite(odom_stamp) &&
      odom_stamp > 0.0 && (now_s - odom_stamp) <= stale_timeout;

  readiness.current_integrity_seen = current_integrity_seen_;
  readiness.current_integrity_valid =
      current_integrity_seen_ && latest_current_valid_;
  readiness.current_integrity_stamp_s = current_stamp;
  readiness.current_integrity_fresh =
      readiness.current_integrity_valid && finite_now &&
      std::isfinite(current_stamp) && current_stamp > 0.0 &&
      (now_s - current_stamp) <= stale_timeout;

  readiness.gnss_epoch_seen = gnss_epoch_seen_;
  readiness.gnss_epoch_valid = gnss_epoch_seen_ && latest_epoch_.has_value() &&
                               std::isfinite(gnss_epoch_stamp) &&
                               epoch_satellite_count > 0;
  readiness.gnss_epoch_stamp_s = gnss_epoch_stamp;
  readiness.gnss_epoch_satellite_count = epoch_satellite_count;
  readiness.gnss_epoch_fresh =
      readiness.gnss_epoch_valid && finite_now &&
      std::isfinite(gnss_epoch_stamp) &&
      (gnss_timeout < 0.0 || (now_s - gnss_epoch_stamp) <= gnss_timeout);

  readiness.origin_seen = origin_seen_;
  readiness.origin_valid = origin_seen_ && origin_set_;
  readiness.origin_stamp_s = origin_stamp;
  readiness.origin_fresh =
      readiness.origin_valid && finite_now && std::isfinite(origin_stamp) &&
      origin_stamp > 0.0 && (now_s - origin_stamp) <= stale_timeout;

  readiness.map_seen = map_seen_;
  readiness.map_valid =
      map_seen_ && std::isfinite(map_stamp) && map_point_count > 0;
  readiness.map_stamp_s = map_stamp;
  readiness.map_point_count = static_cast<uint64_t>(map_point_count);
  readiness.map_fresh =
      readiness.map_valid && finite_now && std::isfinite(map_stamp) &&
      map_stamp > 0.0 && (now_s - map_stamp) <= stale_timeout;
  return readiness;
}

std::string P0RiskGridRuntime::snapshotFailureReason(
    const double now_s) const {
  if (!std::isfinite(now_s) || now_s <= 0.0) {
    return "message_stamp_unavailable";
  }
  const InputReadiness readiness = inputReadiness(now_s);
  if (!readiness.odom_seen) {
    return "odom_missing";
  }
  if (!readiness.odom_valid) {
    return "odom_invalid";
  }
  if (!readiness.current_integrity_seen) {
    return "current_integrity_missing";
  }
  if (!readiness.current_integrity_valid) {
    return "current_integrity_invalid";
  }
  return "none";
}

void P0RiskGridRuntime::recordInputCallback() {
  std::lock_guard<std::mutex> health_lock(health_state_mutex_);
  last_input_callback_steady_s_ = steadyNowSeconds();
  ++input_callback_count_;
}

void P0RiskGridRuntime::odomCallback(
    const nav_msgs::msg::Odometry::ConstSharedPtr msg) {
  if (!msg) {
    return;
  }
  recordInputCallback();
  std::lock_guard<std::mutex> health_lock(health_state_mutex_);
  odom_seen_ = true;
  latest_odom_stamp_ = stampToSec(msg->header.stamp);
  latest_odom_p_ = Eigen::Vector3d(msg->pose.pose.position.x,
                                   msg->pose.pose.position.y,
                                   msg->pose.pose.position.z);
  latest_odom_q_ = Eigen::Quaterniond(msg->pose.pose.orientation.w,
                                      msg->pose.pose.orientation.x,
                                      msg->pose.pose.orientation.y,
                                      msg->pose.pose.orientation.z);
  latest_odom_pose_valid_ =
      latest_odom_p_.allFinite() && std::isfinite(latest_odom_q_.w()) &&
      std::isfinite(latest_odom_q_.x()) &&
      std::isfinite(latest_odom_q_.y()) &&
      std::isfinite(latest_odom_q_.z());
}

void P0RiskGridRuntime::integrityCallback(
    const iap::msg::IntegrityReport::ConstSharedPtr msg) {
  if (!msg) {
    return;
  }
  recordInputCallback();
  std::lock_guard<std::mutex> health_lock(health_state_mutex_);
  advanceNonzeroGeneration(&latest_current_generation_);
  current_integrity_seen_ = true;
  latest_current_ = currentFromMsg(*msg);
  latest_current_valid_ = latest_current_.valid;
}

void P0RiskGridRuntime::rangeCallback(
    const gnss_comm::msg::GnssMeasMsg::ConstSharedPtr msg) {
  if (!msg) {
    return;
  }
  recordInputCallback();

  bool origin_valid = false;
  Eigen::Vector3d origin_ecef = Eigen::Vector3d::Zero();
  std::vector<double> iono_params;
  std::unordered_map<uint32_t, gnss_comm::EphemPtr> ephem_cache;
  std::unordered_map<uint32_t, gnss_comm::GloEphemPtr> glo_ephem_cache;
  {
    std::lock_guard<std::mutex> health_lock(health_state_mutex_);
    origin_valid = origin_set_;
    origin_ecef = origin_ecef_;
    iono_params = iono_params_;
    ephem_cache = ephem_cache_;
    glo_ephem_cache = glo_ephem_cache_;
  }
  const auto obs_list = origin_valid
      ? gnss_comm::msg2meas(msg)
      : decltype(gnss_comm::msg2meas(msg)){};

  iap::GnssEpoch epoch;
  if (!obs_list.empty() && obs_list.front()) {
    const auto utc_t = gnss_comm::gpst2utc(obs_list.front()->time);
    epoch.stamp = static_cast<double>(utc_t.time) + utc_t.sec;
    epoch.gps_sec = static_cast<double>(obs_list.front()->time.time) +
                    obs_list.front()->time.sec;
    epoch.iono_params = iono_params;

    for (const auto& obs : obs_list) {
      if (!obs) {
        continue;
      }
      int l1_idx = -1;
      const double freq = gnss_comm::L1_freq(obs, &l1_idx);
      if (l1_idx < 0 || freq < 0.0 ||
          static_cast<int>(obs->psr.size()) <= l1_idx) {
        continue;
      }
      const double pr = obs->psr[l1_idx];
      if (pr <= 0.0 || !std::isfinite(pr)) {
        continue;
      }

      const uint32_t sat_id = obs->sat;
      const uint32_t sys = gnss_comm::satsys(sat_id, nullptr);
      Eigen::Vector3d sat_ecef_pos = Eigen::Vector3d::Zero();
      Eigen::Vector3d sat_ecef_vel = Eigen::Vector3d::Zero();
      double svdt = 0.0;
      double svddt = 0.0;
      double tgd = 0.0;
      const auto t_tx = gnss_comm::time_add(obs->time, -pr / kLightSpeed);

      if (sys == SYS_GLO) {
        const auto it = glo_ephem_cache.find(sat_id);
        if (it == glo_ephem_cache.end()) {
          continue;
        }
        sat_ecef_pos = gnss_comm::geph2pos(t_tx, it->second, &svdt);
        sat_ecef_vel = gnss_comm::geph2vel(t_tx, it->second, &svddt);
      } else {
        const auto it = ephem_cache.find(sat_id);
        if (it == ephem_cache.end()) {
          continue;
        }
        sat_ecef_pos = gnss_comm::eph2pos(t_tx, it->second, &svdt);
        sat_ecef_vel = gnss_comm::eph2vel(t_tx, it->second, &svddt);
        tgd = it->second->tgd[0];
      }
      if (!sat_ecef_pos.allFinite() || !sat_ecef_vel.allFinite()) {
        continue;
      }

      double azel[2] = {0.0, M_PI / 2.0};
      gnss_comm::sat_azel(origin_ecef, sat_ecef_pos, azel);

      iap::SatObs sat;
      sat.sat_id = static_cast<int>(sat_id);
      sat.constellation = (sys == SYS_GLO) ? 'R'
                          : (sys == SYS_GAL) ? 'E'
                          : (sys == SYS_BDS) ? 'C'
                                             : 'G';
      sat.pr_meas = pr + svdt * kLightSpeed;
      sat.dop_meas = 0.0 + svddt * kLightSpeed;
      sat.pr_sigma =
          static_cast<int>(obs->psr_std.size()) > l1_idx &&
                  obs->psr_std[l1_idx] > 0.05
              ? obs->psr_std[l1_idx]
              : 5.0;
      sat.dop_sigma = 0.5;
      sat.sat_pos = sat_ecef_pos;
      sat.sat_vel = sat_ecef_vel;
      sat.elevation = azel[1];
      sat.azimuth = azel[0];
      sat.tgd = tgd;
      sat.svddt = svddt;
      epoch.sats.push_back(sat);
    }
  }

  {
    std::lock_guard<std::mutex> health_lock(health_state_mutex_);
    advanceNonzeroGeneration(&latest_gnss_epoch_generation_);
    gnss_epoch_seen_ = true;
    latest_gnss_epoch_satellite_count_ =
        static_cast<uint64_t>(epoch.sats.size());
    if (!epoch.sats.empty()) {
      latest_gnss_epoch_stamp_ = epoch.stamp;
      latest_epoch_ = std::move(epoch);
    } else {
      latest_gnss_epoch_stamp_ =
          std::numeric_limits<double>::quiet_NaN();
      latest_epoch_.reset();
    }
  }
}

void P0RiskGridRuntime::ephemCallback(
    const gnss_comm::msg::GnssEphemMsg::ConstSharedPtr msg) {
  if (msg) {
    recordInputCallback();
  }
  auto ephem = gnss_comm::msg2ephem(msg);
  if (ephem) {
    std::lock_guard<std::mutex> lock(health_state_mutex_);
    ephem_cache_[ephem->sat] = ephem;
  }
}

void P0RiskGridRuntime::gloEphemCallback(
    const gnss_comm::msg::GnssGloEphemMsg::ConstSharedPtr msg) {
  if (msg) {
    recordInputCallback();
  }
  auto ephem = gnss_comm::msg2glo_ephem(msg);
  if (ephem) {
    std::lock_guard<std::mutex> lock(health_state_mutex_);
    glo_ephem_cache_[ephem->sat] = ephem;
  }
}

void P0RiskGridRuntime::receiverLlaCallback(
    const sensor_msgs::msg::NavSatFix::ConstSharedPtr msg) {
  if (!msg) {
    return;
  }
  recordInputCallback();
  const double message_stamp = stampToSec(msg->header.stamp);
  std::lock_guard<std::mutex> lock(health_state_mutex_);
  origin_seen_ = true;
  latest_origin_stamp_ = message_stamp;
  if (origin_set_) {
    return;
  }
  if (std::isfinite(msg->latitude) && std::isfinite(msg->longitude) &&
      std::isfinite(msg->altitude)) {
    origin_ecef_ =
        gnss_comm::geo2ecef(Eigen::Vector3d(msg->latitude, msg->longitude,
                                            msg->altitude));
    origin_set_ = true;
  }
}

void P0RiskGridRuntime::ionoCallback(
    const gnss_comm::msg::GnssIonosphereParameter::ConstSharedPtr msg) {
  if (msg) {
    recordInputCallback();
  }
  if (msg && msg->type == 0 && msg->parameters.size() >= 8) {
    std::lock_guard<std::mutex> lock(health_state_mutex_);
    iono_params_.assign(msg->parameters.begin(), msg->parameters.begin() + 8);
  }
}

void P0RiskGridRuntime::cloudCallback(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
  if (msg) {
    recordInputCallback();
  }
  auto clear_lidar_inputs =
      [this](const std::string& reason, const double source_stamp) {
    iap::LidarFimPrimitiveGenerationDiagnostics diagnostics;
    diagnostics.valid = false;
    diagnostics.fallback_reason = reason;
    std::lock_guard<std::mutex> lock(lidar_predictor_input_mutex_);
    latest_lidar_map_points_.reset();
    latest_lidar_fim_primitives_.reset();
    latest_lidar_fim_diagnostics_ = diagnostics;
    latest_lidar_map_point_count_ = 0;
    latest_lidar_fim_primitive_count_ = 0;
    latest_lidar_fim_valid_normal_count_ = 0;
    latest_lidar_fim_fallback_reason_ = reason;
    advanceNonzeroGeneration(&latest_lidar_generation_);
    latest_lidar_stamp_ = source_stamp;
  };

  if (!msg) {
    clear_lidar_inputs("null_lidar_pointcloud",
                       std::numeric_limits<double>::quiet_NaN());
    return;
  }

  const double source_stamp = stampToSec(msg->header.stamp);

  if (!config_.fit_grid_to_map_cloud) {
    std::lock_guard<std::mutex> health_lock(health_state_mutex_);
    map_seen_ = true;
    latest_map_stamp_ = source_stamp;
  }

  auto points = std::make_shared<std::vector<Eigen::Vector3d>>();
  auto normals = std::make_shared<std::vector<Eigen::Vector3d>>();
  const bool cloud_has_normals =
      hasPointField(*msg, "normal_x") &&
      hasPointField(*msg, "normal_y") &&
      hasPointField(*msg, "normal_z");
  try {
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");
    if (cloud_has_normals) {
      sensor_msgs::PointCloud2ConstIterator<float> iter_nx(*msg, "normal_x");
      sensor_msgs::PointCloud2ConstIterator<float> iter_ny(*msg, "normal_y");
      sensor_msgs::PointCloud2ConstIterator<float> iter_nz(*msg, "normal_z");
      for (; iter_x != iter_x.end();
           ++iter_x, ++iter_y, ++iter_z, ++iter_nx, ++iter_ny, ++iter_nz) {
        const Eigen::Vector3d point(*iter_x, *iter_y, *iter_z);
        if (!point.allFinite()) {
          continue;
        }
        const Eigen::Vector3d normal(*iter_nx, *iter_ny, *iter_nz);
        points->push_back(point);
        normals->push_back(normal.allFinite() ? normal
                                              : Eigen::Vector3d::Zero());
      }
    } else {
      for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
        const Eigen::Vector3d point(*iter_x, *iter_y, *iter_z);
        if (point.allFinite()) {
          points->push_back(point);
        }
      }
    }
  } catch (const std::exception& e) {
    clear_lidar_inputs(std::string("invalid_lidar_pointcloud:") + e.what(),
                       source_stamp);
    return;
  }

  if (points->empty()) {
    clear_lidar_inputs("empty_lidar_pointcloud", source_stamp);
    return;
  }

  std::string geometry_reason;
  if (!initializeGridDimensionsFromMapCloud(
          *points, msg->header.frame_id, &geometry_reason)) {
    clear_lidar_inputs("invalid_scene_geometry:" + geometry_reason,
                       source_stamp);
    return;
  }

  const std::shared_ptr<std::vector<Eigen::Vector3d>> predictor_normals =
      cloud_has_normals && normals->size() == points->size() ? normals
                                                             : nullptr;
  iap::LidarFimPrimitiveGenerationDiagnostics diagnostics;
  auto primitives =
      iap::make_lidar_fim_primitives(*points, predictor_normals.get(),
                                     iap::LidarFimPrimitiveGenerationParams{},
                                     &diagnostics);
  const std::size_t primitive_count =
      primitives ? primitives->size() : static_cast<std::size_t>(0);
  const std::size_t valid_normal_count =
      std::max(0, diagnostics.lidar_pca_valid_normals);
  const std::string fallback_reason =
      diagnostics.fallback_reason.empty() ? std::string()
                                          : diagnostics.fallback_reason;
  {
    std::lock_guard<std::mutex> lock(lidar_predictor_input_mutex_);
    latest_lidar_map_points_ = points;
    latest_lidar_fim_primitives_ = primitives;
    latest_lidar_fim_diagnostics_ = diagnostics;
    latest_lidar_map_point_count_ = points->size();
    latest_lidar_fim_primitive_count_ = primitive_count;
    latest_lidar_fim_valid_normal_count_ = valid_normal_count;
    latest_lidar_fim_fallback_reason_ = fallback_reason;
    advanceNonzeroGeneration(&latest_lidar_generation_);
    latest_lidar_stamp_ = source_stamp;
  }
  if (config_.fit_grid_to_map_cloud) {
    std::lock_guard<std::mutex> health_lock(health_state_mutex_);
    map_seen_ = true;
    latest_map_stamp_ = source_stamp;
  }
}

bool P0RiskGridRuntime::initializeGridDimensionsFromMapCloud(
    const std::vector<Eigen::Vector3d>& points,
    const std::string& frame_id,
    std::string* reason) {
  if (!config_.fit_grid_to_map_cloud) {
    if (reason) {
      *reason = "disabled";
    }
    return true;
  }
  if (points.empty() || frame_id != config_.grid.frame_id) {
    if (reason) {
      *reason = points.empty() ? "empty_map_cloud" : "map_frame_mismatch";
    }
    return false;
  }
  {
    std::lock_guard<std::mutex> health_lock(health_state_mutex_);
    if (scene_grid_dimensions_ready_) {
      if (reason) {
        *reason = "already_fitted";
      }
      return true;
    }
  }

  Eigen::Vector3d min_w = points.front();
  Eigen::Vector3d max_w = points.front();
  for (const auto& point : points) {
    if (!point.allFinite()) {
      continue;
    }
    min_w = min_w.cwiseMin(point);
    max_w = max_w.cwiseMax(point);
  }
  const Eigen::Vector3d span = max_w - min_w;
  if (!min_w.allFinite() || !max_w.allFinite() ||
      (span.array() < 0.0).any()) {
    if (reason) {
      *reason = "non_finite_map_bounds";
    }
    return false;
  }

  iap::RiskGridMapParams fitted = config_.grid;
  fitted.size_x_m = std::max(fitted.resolution_m, span.x());
  fitted.size_y_m = std::max(fitted.resolution_m, span.y());
  fitted.size_z_m = std::max(fitted.resolution_m, span.z());
  std::string configure_reason;
  if (!risk_grid_.configure(fitted, &configure_reason)) {
    if (reason) {
      *reason = "risk_grid_configure_failed:" + configure_reason;
    }
    return false;
  }

  {
    std::lock_guard<std::mutex> health_lock(health_state_mutex_);
    scene_grid_dimensions_ready_ = true;
  }
  RCLCPP_INFO(
      node_->get_logger(),
      "[p0] fitted RiskGridMap to scene cloud frame=%s min=(%.3f,%.3f,%.3f) "
      "max=(%.3f,%.3f,%.3f) size=(%.3f,%.3f,%.3f) resolution=%.3f",
      frame_id.c_str(), min_w.x(), min_w.y(), min_w.z(), max_w.x(),
      max_w.y(), max_w.z(), fitted.size_x_m, fitted.size_y_m,
      fitted.size_z_m, fitted.resolution_m);
  if (reason) {
    *reason = "fitted";
  }
  return true;
}

iap::CurrentIntegrityState P0RiskGridRuntime::currentFromMsg(
    const iap::msg::IntegrityReport& msg) const {
  iap::CurrentIntegrityState current;
  current.stamp = stampToSec(msg.header.stamp);
  current.gnss_valid = msg.gnss_valid;
  current.integrity_state = msg.integrity_state;
  current.hpl = msg.hpl;
  current.vpl = msg.vpl;
  current.pl_e = msg.pl_e;
  current.pl_n = msg.pl_n;
  current.pl_u = msg.pl_u;
  current.pl = iap::current_pl_scalar(msg.hpl, msg.vpl);
  current.hal = msg.hal;
  current.val = msg.val;
  current.im = msg.im;
  current.pl_ff = msg.pl_ff;
  current.pl_ff_v = msg.pl_ff_v;
  current.k_ff_used = msg.k_ff_used;
  current.k_fa_used = msg.k_fa_used;
  current.n_sv_used = msg.n_sv_used;
  current.n_constellations = msg.n_constellations;
  current.pdop = msg.pdop;
  current.sigma_h = msg.sigma_h;
  current.n_hypotheses = msg.n_hypotheses;
  current.n_detected = msg.n_detected;
  current.excluded_prns.assign(msg.excluded_prns.begin(),
                               msg.excluded_prns.end());
  current.excluded_trunk_ids.assign(msg.excluded_trunk_ids.begin(),
                                    msg.excluded_trunk_ids.end());
  current.n_trunks_observed = msg.n_trunks_observed;
  current.tdop = msg.tdop;
  current.valid = finite(current.hpl) && finite(current.vpl) &&
                  finite(current.hal) && finite(current.val) &&
                  finite(current.im);
  return current;
}

const iap::GnssEpoch* P0RiskGridRuntime::activeGnssEpoch(
    const double query_stamp) const {
  if (!latest_epoch_) {
    return nullptr;
  }
  const double age_s = query_stamp - latest_epoch_->stamp;
  if (!std::isfinite(age_s)) {
    return nullptr;
  }
  if (config_.gnss_epoch_max_age_s >= 0.0 &&
      age_s > config_.gnss_epoch_max_age_s) {
    return nullptr;
  }
  return &*latest_epoch_;
}

Eigen::Matrix3d P0RiskGridRuntime::currentPriorInformation(
    const iap::CurrentIntegrityState& current) const {
  Eigen::Matrix3d lambda = Eigen::Matrix3d::Zero();
  if (!current.valid) {
    return lambda;
  }
  constexpr double k_h = 5.0;
  constexpr double k_v = 5.0;
  const double sigma_h =
      std::isfinite(current.hpl) && current.hpl > 0.0 ? current.hpl / k_h
                                                       : 0.0;
  const double sigma_v =
      std::isfinite(current.vpl) && current.vpl > 0.0 ? current.vpl / k_v
                                                       : 0.0;
  if (sigma_h > 0.0 && sigma_v > 0.0) {
    lambda(0, 0) = 1.0 / (sigma_h * sigma_h);
    lambda(1, 1) = 1.0 / (sigma_h * sigma_h);
    lambda(2, 2) = 1.0 / (sigma_v * sigma_v);
  }
  return lambda;
}

bool P0RiskGridRuntime::buildSnapshot(
    const double now_s,
    iap::IntegritySnapshot* snapshot,
    PredictorSourceCapture* source_capture,
    InputReadiness* readiness_capture) const {
  if (snapshot == nullptr) {
    return false;
  }
  // Copy a coherent input state before the expensive predictor work starts.
  // Input callbacks may now continue concurrently with refresh.
  double odom_stamp = std::numeric_limits<double>::quiet_NaN();
  Eigen::Vector3d odom_position = Eigen::Vector3d::Zero();
  Eigen::Quaterniond odom_orientation = Eigen::Quaterniond::Identity();
  bool odom_valid = false;
  iap::CurrentIntegrityState current;
  bool current_valid = false;
  uint64_t prior_source_generation = 0;
  Eigen::Matrix3d lambda_prior = Eigen::Matrix3d::Zero();
  std::optional<iap::GnssEpoch> epoch;
  uint64_t captured_gnss_epoch_generation = 0;
  {
    std::scoped_lock lock(health_state_mutex_, lidar_predictor_input_mutex_);
    odom_stamp = latest_odom_stamp_;
    odom_position = latest_odom_p_;
    odom_orientation = latest_odom_q_;
    odom_valid = latest_odom_pose_valid_;
    current = latest_current_;
    current_valid = latest_current_valid_;
    prior_source_generation = latest_current_generation_;
    if (config_.predictor_use_current_integrity_prior) {
      lambda_prior = currentPriorInformation(current);
    }
    epoch = latest_epoch_;
    captured_gnss_epoch_generation = latest_gnss_epoch_generation_;
    if (readiness_capture) {
      *readiness_capture = inputReadinessLocked(now_s);
    }
  }
  if (source_capture) {
    source_capture->current_generation = prior_source_generation;
    source_capture->current_stamp = current.stamp;
    source_capture->gnss_epoch_generation =
        captured_gnss_epoch_generation;
    source_capture->gnss_epoch_stamp =
        epoch ? epoch->stamp : std::numeric_limits<double>::quiet_NaN();
  }
  const bool integrity_epoch_aligned = epoch && current.valid &&
      current.gnss_valid &&
      std::isfinite(current.stamp) && std::isfinite(epoch->stamp) &&
      std::abs(current.stamp - epoch->stamp) <=
          config_.predictor_gnss_measured_epoch_integrity_max_delta_s;
  if (integrity_epoch_aligned && !current.excluded_prns.empty()) {
    const std::unordered_set<int> excluded(
        current.excluded_prns.begin(), current.excluded_prns.end());
    for (auto& sat : epoch->sats) {
      sat.excluded = sat.excluded || excluded.count(sat.sat_id) > 0;
    }
  }
  if (!odom_valid || !current_valid) {
    return false;
  }
  double transaction_now_s = now_s;
  if (std::isfinite(odom_stamp)) {
    transaction_now_s = std::max(transaction_now_s, odom_stamp);
  }
  if (std::isfinite(current.stamp)) {
    transaction_now_s = std::max(transaction_now_s, current.stamp);
  }
  const auto age_valid = [](double stamp, double now_s, double max_age_s) {
    if (!std::isfinite(stamp) || !std::isfinite(now_s) || stamp <= 0.0) {
      return false;
    }
    const double age_s = now_s - stamp;
    if (!std::isfinite(age_s) || age_s < 0.0) {
      return false;
    }
    return max_age_s < 0.0 || age_s <= max_age_s;
  };
  if (!age_valid(odom_stamp, transaction_now_s,
                 config_.grid.stale_timeout_s)) {
    return false;
  }
  if (!age_valid(current.stamp, transaction_now_s,
                 config_.grid.stale_timeout_s)) {
    return false;
  }
  if (epoch && !std::isfinite(epoch->stamp)) {
    return false;
  }
  iap::IntegritySnapshotBuilderInput input;
  input.stamp = transaction_now_s;
  input.has_pose = odom_valid;
  input.pose_stamp = odom_stamp;
  input.p_wb = odom_position;
  input.q_wb = odom_orientation;
  input.current = current;
  if (epoch) {
    const double age_s = transaction_now_s - epoch->stamp;
    if (std::isfinite(age_s) &&
        (config_.gnss_epoch_max_age_s < 0.0 || age_s <= config_.gnss_epoch_max_age_s)) {
      input.gnss_epoch = &*epoch;
    }
  }
  if (config_.predictor_use_current_integrity_prior &&
      lambda_prior.trace() > 0.0 && lambda_prior.allFinite()) {
    input.lambda_base_pos = &lambda_prior;
  }
  *snapshot = snapshot_builder_.build_from_latest(input);
  if (snapshot->has_lambda_base) {
    snapshot->prior_source_generation = prior_source_generation;
  }
  return snapshot->valid;
}

iap::RiskGridHealth P0RiskGridRuntime::addLidarPredictorInputHealth(
    iap::RiskGridHealth health) const {
  std::lock_guard<std::mutex> lock(lidar_predictor_input_mutex_);
  return addLidarPredictorInputHealthLocked(std::move(health));
}

iap::RiskGridHealth P0RiskGridRuntime::addLidarPredictorInputHealthLocked(
    iap::RiskGridHealth health) const {
  health.predictor_lidar_map_point_count =
      static_cast<uint64_t>(latest_lidar_map_point_count_);
  health.predictor_lidar_fim_primitive_count =
      static_cast<uint64_t>(latest_lidar_fim_primitive_count_);
  health.predictor_lidar_fim_valid_normal_count =
      static_cast<uint64_t>(latest_lidar_fim_valid_normal_count_);
  health.predictor_lidar_fim_fallback_reason =
      latest_lidar_fim_fallback_reason_;
  return health;
}

double P0RiskGridRuntime::currentMessageStamp() const {
  std::lock_guard<std::mutex> health_lock(health_state_mutex_);
  if (std::isfinite(latest_odom_stamp_) && latest_odom_stamp_ > 0.0) {
    return latest_odom_stamp_;
  }
  if (std::isfinite(latest_current_.stamp) && latest_current_.stamp > 0.0) {
    return latest_current_.stamp;
  }
  return std::numeric_limits<double>::quiet_NaN();
}

double P0RiskGridRuntime::liveNowSeconds() const {
  const double message_stamp_s = currentMessageStamp();
  const bool use_sim_time =
      node_ && node_->has_parameter("use_sim_time") &&
      node_->get_parameter("use_sim_time").as_bool();
  if (use_sim_time) {
    const double now_s = node_->now().seconds();
    if (std::isfinite(now_s) && now_s > 0.0) {
      // ROS delivery can expose a sensor/odom message a few milliseconds
      // before the matching /clock sample. Keep one message time domain and
      // never classify that valid newest input as being from the future.
      return std::isfinite(message_stamp_s)
          ? std::max(now_s, message_stamp_s)
          : now_s;
    }
  }
  return message_stamp_s;
}

double P0RiskGridRuntime::currentRefreshStamp() const {
  return currentMessageStamp();
}

}  // namespace ego_planner
