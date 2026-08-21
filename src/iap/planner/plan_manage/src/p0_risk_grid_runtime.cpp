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

struct PredictorSpatialSourceProjection {
  bool gnss = false;
  bool lidar = false;
  bool legacy_lidar = false;
};

PredictorSpatialSourceProjection predictorSpatialSourceProjection(
    const iap::PredictorParams& params) {
  PredictorSpatialSourceProjection projection;
  projection.gnss =
      params.source_mode != iap::PredictorSourceMode::LidarOnly &&
      params.gnss_epoch_policy != iap::PredictorGnssEpochPolicy::Disabled;
  projection.lidar =
      params.source_mode != iap::PredictorSourceMode::GnssOnly;
  projection.legacy_lidar =
      projection.lidar && params.lidar.enable_legacy_observability;
  return projection;
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
      std::uint64_t occupancy_generation,
      std::shared_ptr<const std::vector<Eigen::Vector3d>> lidar_map_points,
      std::shared_ptr<const std::vector<iap::LidarFimPrimitive>>
          lidar_fim_primitives,
      iap::PredictorModule module,
      iap::IntegritySnapshot snapshot,
      int worker_count)
      : occupancy_owner_(std::move(occupancy_owner)),
        rolling_window_(rolling_window), snapshot_(std::move(snapshot)),
        worker_count_(std::max(1, worker_count)) {
    iap::RollingSpatialRefreshInput input;
    input.geometry = std::move(geometry);
    input.module = std::move(module);
    input.snapshot = snapshot_;
    input.occupancy_owner = occupancy_owner_;
    input.occupancy_generation = occupancy_generation;
    input.lidar_map_points_owner = std::move(lidar_map_points);
    input.lidar_fim_primitives_owner = std::move(lidar_fim_primitives);
    ready_ = rolling_window_ &&
             rolling_window_->beginRefresh(std::move(input));
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
                    iap::makeRiskPredictionResult(predictions[local]);
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
    return rolling_window_ ? rolling_window_->diagnostics()
                           : iap::RollingSpatialRefreshDiagnostics{};
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
  bool ready_ = false;
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
  config.grid.horizons_s = node->declare_parameter<std::vector<double>>(
      "p0.horizons_s", std::vector<double>{0.0, 0.5, 1.0, 1.5, 2.0});
  config.grid.refresh_period_s =
      node->declare_parameter<double>("p0.refresh_period_s", 0.5);
  config.grid.stale_timeout_s =
      node->declare_parameter<double>("p0.stale_timeout_s", 1.0);
  config.grid.skip_occupied_voxels =
      node->declare_parameter<bool>("p0.skip_occupied_voxels",
                                    config.grid.skip_occupied_voxels);
  config.debug_metrics_enable =
      node->declare_parameter<bool>("p0.debug_metrics_enable", false);
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
      node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  refresh_callback_group_ =
      node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  health_callback_group_ =
      node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  rclcpp::SubscriptionOptions subscription_options;
  subscription_options.callback_group = input_callback_group_;
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
      subscription_options);
  ephem_sub_ = node_->create_subscription<gnss_comm::msg::GnssEphemMsg>(
      config_.ephem_topic, qos,
      [this](const gnss_comm::msg::GnssEphemMsg::ConstSharedPtr msg) {
        ephemCallback(msg);
      },
      subscription_options);
  glo_ephem_sub_ =
      node_->create_subscription<gnss_comm::msg::GnssGloEphemMsg>(
          config_.glo_ephem_topic, qos,
          [this](const gnss_comm::msg::GnssGloEphemMsg::ConstSharedPtr msg) {
            gloEphemCallback(msg);
          },
          subscription_options);
  receiver_lla_sub_ =
      node_->create_subscription<sensor_msgs::msg::NavSatFix>(
          config_.receiver_lla_topic, qos,
          [this](const sensor_msgs::msg::NavSatFix::ConstSharedPtr msg) {
            receiverLlaCallback(msg);
          },
          subscription_options);
  iono_sub_ =
      node_->create_subscription<gnss_comm::msg::GnssIonosphereParameter>(
          config_.iono_topic, qos,
          [this](const gnss_comm::msg::GnssIonosphereParameter::ConstSharedPtr msg) {
            ionoCallback(msg);
          },
          subscription_options);
  cloud_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
      config_.map_topic, rclcpp::QoS(2).transient_local().reliable(),
      [this](const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
        cloudCallback(msg);
      },
      subscription_options);

  // Do not couple the machine-readable health contract to RViz/debug flags.
  health_pub_ = node_->create_publisher<std_msgs::msg::String>(
      "/planning/risk_grid_health", 10);
  safety_viz_ = std::make_shared<SafetyRvizPublisher>(
      node_, SafetyRvizPublisher::declareAndReadConfig(node_));
  const double period_s =
      std::max(0.001, config_.grid.refresh_period_s);
  {
    std::lock_guard<std::mutex> lock(health_state_mutex_);
    next_refresh_scheduled_steady_s_ = steadyNowSeconds() + period_s;
  }
  refresh_timer_ = node_->create_wall_timer(
      std::chrono::duration<double>(period_s),
      [this]() { refreshTimerCallback(); },
      refresh_callback_group_);
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

void P0RiskGridRuntime::refreshTimerCallback() {
  if (!config_.enable_risk_grid) {
    return;
  }
  const auto refresh_start = std::chrono::steady_clock::now();
  const double refresh_start_steady_s = steadyNowSeconds();
  const double now_s = liveNowSeconds();
  const std::string pre_build_failure = snapshotFailureReason(now_s);
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
    last_predictor_spatial_advisory_recompute_count_ = 0;
    last_predictor_spatial_advisory_reuse_count_ = 0;
    last_predictor_gnss_advisory_invocation_count_ = 0;
    last_predictor_lidar_advisory_invocation_count_ = 0;
    last_predictor_horizon_fusion_count_ = 0;
    last_rolling_spatial_diagnostics_ = {};
    last_provider_batch_duration_ms_ =
        std::numeric_limits<double>::quiet_NaN();
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
    return;
  }
  const auto fail_semantic_refresh = [&](const P0SemanticFailure failure) {
    const std::string failure_reason = semanticFailureReason(failure);
    risk_grid_.markRefreshFailure(now_s, failure_reason);
    const double refresh_end_stamp_s = liveNowSeconds();
    std::lock_guard<std::mutex> health_lock(health_state_mutex_);
    last_snapshot_failure_reason_ = failure_reason;
    last_refresh_succeeded_ = false;
    last_refresh_elapsed_ms_ = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - refresh_start).count();
    last_refresh_end_stamp_s_ = refresh_end_stamp_s;
    last_refresh_end_steady_s_ = steadyNowSeconds();
  };
  const bool production_predictor = provider_ == nullptr;
  if (production_predictor && config_.predictor_use_current_integrity_prior) {
    const InputReadiness readiness = inputReadiness(now_s);
    if (readiness.current_integrity_seen &&
        readiness.current_integrity_valid &&
        !readiness.current_integrity_fresh) {
      fail_semantic_refresh(P0SemanticFailure::STALE_COVARIANCE_GROWTH_PRIOR);
      return;
    }
  }
  iap::IntegritySnapshot snapshot;
  uint64_t captured_gnss_epoch_generation = 0;
  if (!buildSnapshot(now_s, &snapshot,
                     &captured_gnss_epoch_generation)) {
    risk_grid_.markRefreshFailure(now_s, "snapshot_unavailable");
    const double refresh_end_stamp_s = liveNowSeconds();
    {
      std::lock_guard<std::mutex> health_lock(health_state_mutex_);
      last_refresh_elapsed_ms_ = std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - refresh_start).count();
      last_refresh_end_stamp_s_ = refresh_end_stamp_s;
      last_refresh_end_steady_s_ = steadyNowSeconds();
      if (pre_build_failure == "none") {
        last_snapshot_failure_reason_ = "snapshot_builder_invalid";
      }
    }
    publishHealth(risk_grid_.health(now_s), now_s);
    return;
  }
  {
    std::lock_guard<std::mutex> health_lock(health_state_mutex_);
    last_snapshot_available_ = true;
    last_snapshot_failure_reason_ = "none";
  }

  std::optional<P0OccupancyEpoch> occupancy_epoch;
  if (occupancy_epoch_factory_) {
    P0OccupancyEpochCapture capture = occupancy_epoch_factory_();
    if (capture.status ==
        P0OccupancyEpochCaptureStatus::SNAPSHOT_UNAVAILABLE) {
      fail_semantic_refresh(
          P0SemanticFailure::OCCUPANCY_SNAPSHOT_UNAVAILABLE);
      return;
    }
    if (capture.status == P0OccupancyEpochCaptureStatus::ADAPTER_INVALID ||
        !capture.epoch.has_value()) {
      fail_semantic_refresh(P0SemanticFailure::OCCUPANCY_LOS_ADAPTER_INVALID);
      return;
    }
    occupancy_epoch = std::move(capture.epoch);
    if (!occupancy_epoch->diagnostic_query ||
        !occupancy_epoch->los_owner || !occupancy_epoch->live_generation ||
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
      fail_semantic_refresh(P0SemanticFailure::OCCUPANCY_STALE);
      return;
    }
    if (occupancy_epoch->live_generation() != occupancy_epoch->generation) {
      fail_semantic_refresh(
          P0SemanticFailure::OCCUPANCY_GENERATION_CHANGED);
      return;
    }
    if (production_predictor && rolling_occupancy_owner_ &&
        rolling_occupancy_generation_ == occupancy_epoch->generation) {
      occupancy_epoch->los_owner = rolling_occupancy_owner_;
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
  if (production_predictor && snapshot.prior_source_generation == 0u) {
    fail_semantic_refresh(P0SemanticFailure::MISSING_COVARIANCE_GROWTH_PRIOR);
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
  std::shared_ptr<const std::vector<Eigen::Vector3d>>
      captured_lidar_map_points;
  std::shared_ptr<const std::vector<iap::LidarFimPrimitive>>
      captured_lidar_fim_primitives;
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
    predictor_params.gnss_epoch_policy =
        config_.predictor_gnss_epoch_policy;
    predictor_params.fusion.conservative_max_with_gnss =
        config_.predictor_conservative_max_with_gnss;
    predictor_params.lidar.enable_legacy_observability =
        config_.predictor_lidar_legacy_observability;
    predictor_params.covariance_growth.sigma_grow_m_sqrt_s =
        config_.predictor_sigma_grow_m_sqrt_s;
    const PredictorSpatialSourceProjection source_projection =
        predictorSpatialSourceProjection(predictor_params);
    validate_gnss_spatial_source = source_projection.gnss;
    validate_lidar_spatial_source = source_projection.lidar;
    validate_lidar_legacy_source = source_projection.legacy_lidar;
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
    }
    captured_lidar_map_points = lidar_map_points;
    captured_lidar_fim_primitives = lidar_fim_primitives;
    iap::PredictorModule module(predictor_params);
    module.set_lidar_map_points(lidar_map_points);
    module.set_lidar_fim_primitives(lidar_fim_primitives);
    iap::RollingSpatialWindowGeometry rolling_geometry;
    rolling_geometry.frame_id = config_.grid.frame_id;
    rolling_geometry.lattice_anchor_w = config_.grid.lattice_anchor_w;
    rolling_geometry.resolution_m = config_.grid.resolution_m;
    rolling_geometry.shape = risk_grid_.voxelNum();
    auto owned_predictor_provider = std::make_unique<PredictorModuleRiskProvider>(
        &rolling_spatial_window_, std::move(rolling_geometry),
        occupancy_epoch->los_owner, occupancy_epoch->generation,
        std::move(lidar_map_points), std::move(lidar_fim_primitives),
        std::move(module), snapshot,
        config_.predictor_effective_worker_count);
    predictor_provider = owned_predictor_provider.get();
    local_provider = std::move(owned_predictor_provider);
    provider = local_provider.get();
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
  const auto occupancy_diagnostic_query = combinedOccupancyDiagnosticQuery(
      occupancy_epoch ? occupancy_epoch->diagnostic_query
                      : iap::RiskGridMap::OccupancyDiagnosticQuery{});
  const auto occupancy_predicate = combinedOccupancyPredicate();
  iap::RiskGridMap::SourceValidator source_validator;
  if (occupancy_epoch) {
    const auto live_occupancy_generation = occupancy_epoch->live_generation;
    const uint64_t captured_occupancy_generation =
        occupancy_epoch->generation;
    const uint64_t captured_prior_generation =
        snapshot.prior_source_generation;
    source_validator =
        [this, live_occupancy_generation, captured_occupancy_generation,
         captured_prior_generation, validate_gnss_spatial_source,
         validate_lidar_spatial_source, validate_lidar_legacy_source,
         captured_gnss_epoch_generation,
         captured_lidar_map_points, captured_lidar_fim_primitives]() {
          if (!live_occupancy_generation ||
              live_occupancy_generation() !=
                  captured_occupancy_generation) {
            return iap::RiskGridSourceValidation::
                OCCUPANCY_GENERATION_CHANGED;
          }
          uint64_t live_prior_generation = 0;
          {
            std::lock_guard<std::mutex> lock(health_state_mutex_);
            live_prior_generation = latest_current_generation_;
          }
          if (captured_prior_generation == 0u ||
              live_prior_generation != captured_prior_generation) {
            return iap::RiskGridSourceValidation::PRIOR_GENERATION_CHANGED;
          }
          if (validate_gnss_spatial_source) {
            std::lock_guard<std::mutex> lock(health_state_mutex_);
            if (latest_gnss_epoch_generation_ !=
                captured_gnss_epoch_generation) {
              return iap::RiskGridSourceValidation::
                  PREDICTOR_SPATIAL_SOURCE_CHANGED;
            }
          }
          if (validate_lidar_spatial_source) {
            std::lock_guard<std::mutex> lock(lidar_predictor_input_mutex_);
            if (!sameSharedOwner(latest_lidar_fim_primitives_,
                                 captured_lidar_fim_primitives)) {
              return iap::RiskGridSourceValidation::
                  PREDICTOR_SPATIAL_SOURCE_CHANGED;
            }
            if (validate_lidar_legacy_source &&
                !sameSharedOwner(latest_lidar_map_points_,
                                 captured_lidar_map_points)) {
              return iap::RiskGridSourceValidation::
                  PREDICTOR_SPATIAL_SOURCE_CHANGED;
            }
          }
          return iap::RiskGridSourceValidation::VALID;
        };
  }
  TimedRiskProvider timed_provider(provider);
  bool refresh_succeeded = false;
  if (occupancy_diagnostic_query) {
    refresh_succeeded = risk_grid_.refreshFromProvider(
        snapshot.p_wb, now_s, timed_provider,
        occupancy_diagnostic_query, source_validator, &reason);
  } else {
    refresh_succeeded = risk_grid_.refreshFromProvider(
        snapshot.p_wb, now_s, timed_provider, occupancy_predicate, &reason);
  }
  const iap::RiskGridHealth health = risk_grid_.health(now_s);
  const auto viz_snapshot = risk_grid_.acquireSnapshot();
  const double refresh_end_stamp_s = liveNowSeconds();
  iap::RollingSpatialRefreshDiagnostics rolling_diagnostics;
  if (predictor_provider) {
    rolling_diagnostics = predictor_provider->rollingDiagnostics();
    predictor_provider->finish(refresh_succeeded);
    if (refresh_succeeded && occupancy_epoch) {
      rolling_occupancy_owner_ = occupancy_epoch->los_owner;
      rolling_occupancy_generation_ = occupancy_epoch->generation;
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
    last_generation_interval_ms_ = std::isfinite(last_generation_end_steady_s_)
        ? 1000.0 * (last_refresh_end_steady_s_ -
                    last_generation_end_steady_s_)
        : std::numeric_limits<double>::quiet_NaN();
    last_generation_end_steady_s_ = last_refresh_end_steady_s_;
  }
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
  const InputReadiness readiness = inputReadiness(now_s);
  const auto mutex_wait_start = std::chrono::steady_clock::now();
  HealthPublicationState state;
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
    state = healthPublicationStateSnapshot();
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
    snapshot_failure_reason = last_snapshot_failure_reason_;
  }
  if (safety_viz_) {
    safety_viz_->publishRiskGridHealth(enriched_health, now_s);
  }
  if (!node_ || !health_pub_) {
    return;
  }
  const auto& out_health = enriched_health;
  std::ostringstream oss;
  oss << "{"
      << "\"ready\":" << (out_health.ready ? "true" : "false") << ","
      << "\"stale\":" << (out_health.stale ? "true" : "false") << ","
      << "\"age_s\":" << jsonNumber(out_health.age_s) << ","
      << "\"valid_ratio\":" << jsonNumber(out_health.valid_ratio) << ","
      << "\"unknown_ratio\":" << jsonNumber(out_health.unknown_ratio) << ","
      << "\"generation_id\":" << out_health.generation_id << ","
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
  state.predictor_spatial_invalidation_reason =
      iap::rollingSpatialInvalidationReasonName(
          last_rolling_spatial_diagnostics_.invalidation_reason);
  state.input_callback_count = input_callback_count_;
  state.health_callback_count = health_callback_count_;
  state.snapshot_available = last_snapshot_available_;
  return state;
}

P0RiskGridRuntime::InputReadiness
P0RiskGridRuntime::inputReadiness(const double now_s) const {
  InputReadiness readiness;
  double odom_stamp = std::numeric_limits<double>::quiet_NaN();
  double current_stamp = std::numeric_limits<double>::quiet_NaN();
  double map_stamp = std::numeric_limits<double>::quiet_NaN();
  double origin_stamp = std::numeric_limits<double>::quiet_NaN();
  double gnss_epoch_stamp = std::numeric_limits<double>::quiet_NaN();
  bool odom_seen = false;
  bool odom_valid = false;
  bool current_seen = false;
  bool current_valid = false;
  bool epoch_seen = false;
  uint64_t epoch_satellite_count = 0;
  bool origin_seen = false;
  bool map_seen = false;
  {
    std::lock_guard<std::mutex> health_lock(health_state_mutex_);
    odom_seen = odom_seen_;
    odom_valid = latest_odom_pose_valid_;
    odom_stamp = latest_odom_stamp_;
    current_seen = current_integrity_seen_;
    current_valid = latest_current_valid_;
    current_stamp = latest_current_.stamp;
    epoch_seen = gnss_epoch_seen_;
    gnss_epoch_stamp = latest_gnss_epoch_stamp_;
    epoch_satellite_count = latest_gnss_epoch_satellite_count_;
    if (latest_epoch_) {
      gnss_epoch_stamp = latest_epoch_->stamp;
      epoch_satellite_count =
          static_cast<uint64_t>(latest_epoch_->sats.size());
    }
    origin_seen = origin_seen_;
    origin_stamp = latest_origin_stamp_;
    map_seen = map_seen_;
    map_stamp = latest_map_stamp_;
  }
  std::size_t map_point_count = 0;
  {
    std::lock_guard<std::mutex> lidar_lock(lidar_predictor_input_mutex_);
    map_point_count = latest_lidar_map_point_count_;
  }

  const double stale_timeout = config_.grid.stale_timeout_s;
  const double gnss_timeout = config_.gnss_epoch_max_age_s;
  const bool finite_now = std::isfinite(now_s);

  readiness.odom_seen = odom_seen;
  readiness.odom_valid = odom_seen && odom_valid;
  readiness.odom_stamp_s = odom_stamp;
  readiness.odom_fresh =
      readiness.odom_valid && finite_now && std::isfinite(odom_stamp) &&
      odom_stamp > 0.0 && (now_s - odom_stamp) <= stale_timeout;

  readiness.current_integrity_seen = current_seen;
  readiness.current_integrity_valid = current_seen && current_valid;
  readiness.current_integrity_stamp_s = current_stamp;
  readiness.current_integrity_fresh =
      readiness.current_integrity_valid && finite_now &&
      std::isfinite(current_stamp) && current_stamp > 0.0 &&
      (now_s - current_stamp) <= stale_timeout;

  readiness.gnss_epoch_seen = epoch_seen;
  readiness.gnss_epoch_valid = epoch_seen && latest_epoch_.has_value() &&
                               std::isfinite(gnss_epoch_stamp) &&
                               epoch_satellite_count > 0;
  readiness.gnss_epoch_stamp_s = gnss_epoch_stamp;
  readiness.gnss_epoch_satellite_count = epoch_satellite_count;
  readiness.gnss_epoch_fresh =
      readiness.gnss_epoch_valid && finite_now &&
      std::isfinite(gnss_epoch_stamp) &&
      (gnss_timeout < 0.0 || (now_s - gnss_epoch_stamp) <= gnss_timeout);

  readiness.origin_seen = origin_seen;
  readiness.origin_valid = origin_seen && origin_set_;
  readiness.origin_stamp_s = origin_stamp;
  readiness.origin_fresh =
      readiness.origin_valid && finite_now && std::isfinite(origin_stamp) &&
      origin_stamp > 0.0 && (now_s - origin_stamp) <= stale_timeout;

  readiness.map_seen = map_seen;
  readiness.map_valid =
      map_seen && std::isfinite(map_stamp) && map_point_count > 0;
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
  ++latest_current_generation_;
  if (latest_current_generation_ == 0u) {
    ++latest_current_generation_;
  }
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
    gnss_epoch_seen_ = true;
    latest_gnss_epoch_stamp_ = std::numeric_limits<double>::quiet_NaN();
    origin_valid = origin_set_;
    origin_ecef = origin_ecef_;
    iono_params = iono_params_;
    ephem_cache = ephem_cache_;
    glo_ephem_cache = glo_ephem_cache_;
  }
  if (!origin_valid) {
    std::lock_guard<std::mutex> health_lock(health_state_mutex_);
    latest_gnss_epoch_satellite_count_ = 0;
    return;
  }

  const auto obs_list = gnss_comm::msg2meas(msg);
  if (obs_list.empty()) {
    std::lock_guard<std::mutex> health_lock(health_state_mutex_);
    latest_gnss_epoch_satellite_count_ = 0;
    return;
  }

  iap::GnssEpoch epoch;
  const auto utc_t = gnss_comm::gpst2utc(obs_list.front()->time);
  epoch.stamp = static_cast<double>(utc_t.time) + utc_t.sec;
  const double gnss_epoch_stamp = epoch.stamp;
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

  {
    std::lock_guard<std::mutex> health_lock(health_state_mutex_);
    latest_gnss_epoch_stamp_ = gnss_epoch_stamp;
    latest_gnss_epoch_satellite_count_ =
        static_cast<uint64_t>(epoch.sats.size());
  }
  if (!epoch.sats.empty()) {
    std::lock_guard<std::mutex> health_lock(health_state_mutex_);
    latest_epoch_ = std::move(epoch);
    ++latest_gnss_epoch_generation_;
    if (latest_gnss_epoch_generation_ == 0u) {
      ++latest_gnss_epoch_generation_;
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
  auto clear_lidar_inputs = [this](const std::string& reason) {
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
  };

  if (!msg) {
    clear_lidar_inputs("null_lidar_pointcloud");
    return;
  }

  {
    std::lock_guard<std::mutex> health_lock(health_state_mutex_);
    map_seen_ = true;
  }
  {
    std::lock_guard<std::mutex> health_lock(health_state_mutex_);
    latest_map_stamp_ = stampToSec(msg->header.stamp);
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
    clear_lidar_inputs(std::string("invalid_lidar_pointcloud:") + e.what());
    return;
  }

  if (points->empty()) {
    clear_lidar_inputs("empty_lidar_pointcloud");
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
  }
}

iap::CurrentIntegrityState P0RiskGridRuntime::currentFromMsg(
    const iap::msg::IntegrityReport& msg) const {
  iap::CurrentIntegrityState current;
  current.stamp = stampToSec(msg.header.stamp);
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
    uint64_t* gnss_epoch_generation) const {
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
    std::lock_guard<std::mutex> lock(health_state_mutex_);
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
  }
  if (gnss_epoch_generation) {
    *gnss_epoch_generation = captured_gnss_epoch_generation;
  }
  if (!odom_valid || !current_valid) {
    return false;
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
  if (!age_valid(odom_stamp, now_s, config_.grid.stale_timeout_s)) {
    return false;
  }
  if (!age_valid(current.stamp, now_s, config_.grid.stale_timeout_s)) {
    return false;
  }
  iap::IntegritySnapshotBuilderInput input;
  input.stamp = now_s;
  input.has_pose = odom_valid;
  input.pose_stamp = odom_stamp;
  input.p_wb = odom_position;
  input.q_wb = odom_orientation;
  input.current = current;
  if (epoch) {
    const double age_s = now_s - epoch->stamp;
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
  const bool use_sim_time =
      node_ && node_->has_parameter("use_sim_time") &&
      node_->get_parameter("use_sim_time").as_bool();
  if (use_sim_time) {
    const double now_s = node_->now().seconds();
    if (std::isfinite(now_s) && now_s > 0.0) {
      return now_s;
    }
  }
  return currentMessageStamp();
}

double P0RiskGridRuntime::currentRefreshStamp() const {
  return currentMessageStamp();
}

}  // namespace ego_planner
